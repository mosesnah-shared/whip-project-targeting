import copy
import torch
import torch.autograd
import torch.nn            as nn
import torch.nn.functional as F 
import torch.optim         as optim
import numpy               as np


from typing import Union, List


__all__ = [ "ReplayBuffer", "OUNoise", "DDPG", "TD3" ]

# Check whether GPU computation (i.e., CUDA) is available.
device = torch.device( "cuda" if torch.cuda.is_available( ) else "cpu" )


class NeuralNetwork( nn.Module ):
    def __init__( self, n_input: int, n_output: int, n_hidden: Union[ int, List[ int] ], act_funcs: List[ str ], gain: float = 1.0 ):
        """
            Define a Neural network which maps from n_input to n_output.  
        """

        # Class inheritance. 
        super( NeuralNetwork, self ).__init__( )

        # Saving the arguments in the class instance. 
        # The number of input neurons 
        self.n_input = n_input

        # The list of numbers of the hidden layer, it is a list since there could be multiple hidden neurons
        self.n_hidden = [ n_hidden ] if isinstance( n_hidden, int ) else n_hidden

        # The number of output neurons
        self.n_output = n_output

        # Saving a list of numbers of layer 
        self.list_n = [ self.n_input ] + self.n_hidden + [ self.n_output ]

        # A string list of activation functions, e.g., ReLU, tanh etc. 
        assert len( act_funcs ) == len( self.list_n ) - 1
        self.act_funcs = act_funcs

        # The gain that will multiply the output value
        self.gain = gain

        # Saving the layers in the list 
        # For this case, we only use Linear combination of neural network. 
        # Iterating through the (current, next ) of the list of numbers 
        # This connects the adjacent pair of layers 
        self.layers = nn.ModuleList( )
        for n_current, n_next in zip(  self.list_n[ : -1 ], self.list_n[ 1 : ] ):
            self.layers.append( nn.Linear( n_current, n_next ) ) 

    def __len__( self ):
        """
            Return the number of layers of this current neural network
        """
        return len( self.list_n )

    def forward( self, input ):
        
        # Just changing the name to x just for brevity
        x = input 

        # Forward the network.
        # If there is no activation function, just apply a linear layer 
        for idx, func in enumerate( self.act_funcs ):
            # getattr does torch.{func}(  )
            x = getattr( torch, func )( self.layers[ idx ]( x ) ) if func is not None else self.layers[ idx ]( x ) 

        return x * self.gain

class ReplayBuffer:

    def __init__( self, n_state, n_action, max_size = 100000 ):

        # Save the dimension of state, dimension of action and the maximum size of the replay buffer
        self.n_state  = n_state
        self.n_action = n_action
        self.max_size = max_size

        # Defining the current size of the replay buffer, just to make the sampling easy. 
        self.current_size = 0

        # Defining the Index Pointer (ptr) of the replaybuffer. 
        # This is required for "adding" the experiences on the replaybuffer. 
        # Using this Index Pointer will become handy and we do not need to call "deque" class. 
        self.idx_ptr = 0

        # Defining the 2D arrays of the ReplayBuffer
        # 2D array definition is necessary to forward the Neural Network
        self.states      = np.zeros( ( max_size, n_state   ) )
        self.actions     = np.zeros( ( max_size, n_action  ) )
        self.rewards     = np.zeros( ( max_size, 1         ) )
        self.next_states = np.zeros( ( max_size, n_state   ) )
        self.is_done     = np.zeros( ( max_size, 1         ) )


    def add( self, state, action, reward, next_state, is_done ):
        """
            Adding a state-action-reward-next_state pair into the ReplayBuffer. 
        """

        self.states[      self.idx_ptr ] = state
        self.actions[     self.idx_ptr ] = action
        self.rewards[     self.idx_ptr ] = reward
        self.next_states[ self.idx_ptr ] = next_state
        self.is_done[     self.idx_ptr ] = is_done

        # Update our index pointer. Note that the "oldest" experiences are overwritten.
        self.idx_ptr = ( self.idx_ptr + 1 ) % self.max_size

        # Update the current size of the replay buffer
        self.current_size = min( self.current_size + 1, self.max_size )

    def sample( self, n_batch_size ):
        """
            Collect "n_batch_size" samples from the replay buffer and return it as a batch.
        """
        idx = np.random.randint( 0, self.current_size, size = n_batch_size )

        # Returning the 2D numpy array as a 2D torch array.

        return ( 
            torch.FloatTensor(      self.states[ idx ]  ).to( device ) ,
            torch.FloatTensor(     self.actions[ idx ]  ).to( device ) , 
            torch.FloatTensor(     self.rewards[ idx ]  ).to( device ) ,
            torch.FloatTensor( self.next_states[ idx ]  ).to( device ) ,
            torch.FloatTensor(     self.is_done[ idx ]  ).to( device )   
        )


    def reset( self ):
        """
            Reset all the replay buffers to zeros
        """
        self.states      = np.zeros( ( self.max_size, self.n_state   ) )
        self.actions     = np.zeros( ( self.max_size, self.n_action  ) )
        self.rewards     = np.zeros( ( self.max_size, 1              ) )
        self.next_states = np.zeros( ( self.max_size, self.n_state   ) )
        self.is_done     = np.zeros( ( self.max_size, 1              ) )

class OUNoise:
    
    def __init__( self, n_action: int, lb_act: np.ndarray, ub_act: np.ndarray , mu = 0.0, theta = 0.15, max_sigma = 0.3, min_sigma = 0.3, decay_period = 100000 ):
        self.mu           = mu
        self.theta        = theta
        self.sigma        = max_sigma
        self.max_sigma    = max_sigma
        self.min_sigma    = min_sigma
        self.decay_period = decay_period
        self.action_dim   = n_action

        # The dimension (length) of the upper/lower bound should match the action space dimension
        assert len( lb_act ) == n_action and len( ub_act ) == n_action
        self.action_lb  = lb_act
        self.action_ub  = ub_act
        self.reset( )
        
    def reset( self ):
        # Caution! The state here is not the "pendulum"'s state, but the "noise" itself. 
        self.state = np.ones( self.action_dim ) * self.mu
        
    def step( self, t = 0 ):
        
        # Call the current noise value. 
        x  = self.state

        # randn returns a sample from the standard (i.e., normal) distribution
        dx = self.theta * ( self.mu - x ) + self.sigma * np.random.randn( self.action_dim )

        # For our case, we simply set the max_sigma and min_sigma the same, hence the sigma value is constant for us
        self.sigma = self.max_sigma - ( self.max_sigma - self.min_sigma ) * min( 1.0, t / self.decay_period )

        # Time-increment. x_{n+1} = x_{n} + dx
        self.state = x + dx

        return self.state
    
    def add_noise2action( self, action, t = 0 ): 
        
        # Calculate the noise with respect to the given time. 
        ou_noise   = self.step( t )

        # Adding ou noise onto the action and then clipping it.
        return np.clip( action + ou_noise, self.action_lb, self.action_ub )



class DDPG:

    def __init__( self, n_state, n_action, n_hidden, act_funcs_actor, act_funcs_critic, gain = 1., gamma = 0.99, tau = 0.005, batch_size = 256 ):

        # Actor Network, its target (copy) Network, and the ADAM optimizer.
        self.actor             = NeuralNetwork( n_input = n_state, n_output = n_action, n_hidden = n_hidden, gain = gain, act_funcs = act_funcs_actor ).to( device )
        self.actor_target      = copy.deepcopy( self.actor )
        self.actor_optimizer   = optim.Adam( self.actor.parameters( ), lr = 1e-4 )

        # Critic Network, its target (copy) Network, and the ADAM optimizer.
        # Learning The Q(s,a) scalar function
        self.critic            = NeuralNetwork( n_input = n_state + n_action, n_output = 1, n_hidden = n_hidden, gain = 1, act_funcs = act_funcs_critic  ).to( device )
        self.critic_target     = copy.deepcopy( self.critic )
        self.critic_optimizer  = optim.Adam( self.critic.parameters( ), lr = 1e-3 )

        # The discount factor gamma and the soft-update gain tau
        self.gamma = gamma
        self.tau   = tau

        # The gain that will be multipled to the output. 
        self.gain = gain

        # The number of batch_size for the update 
        self.n_batch = batch_size

    
    def get_action( self, state ):
        """
            This is a deterministic function, hence mapping from state s to a, a = mu( s )
        """

        # Conduct the a = mu(s), where mu is a "deterministic function"
        # Unsqueeze makes an 1 x n_s array of state. 
        state  = torch.from_numpy( state ).float( ).unsqueeze( 0 ).to( device )

        # Returns an 1 x n_a array of state
        # forward method can be omitted
        action = self.actor( state )

        # Change action from Torch to Numpy.
        # Since n_a is 1 for this case, action is simply an 1x1 array.
        # Hence, flattening the data. 
        action = action.cpu( ).data.numpy( ).flatten( )

        return action
    
    def update( self, replay_buffer, batch_size = 256 ):
        """
            Mini-batch update. 
        """
        # Randomly sample batch_size numbers of S A R S.
        states, actions, rewards, next_states, is_done = replay_buffer.sample( self.n_batch )

        # ====================================================== #
        # ================ Critic Optimizer Part =============== #
        # ====================================================== #
        # Keep in mind that the original paper first optimizes the Critic network
        # Critic loss 
        
        Qprime  = self.critic_target( torch.cat( [ next_states, self.actor_target( next_states ) ], dim = 1 ) )
        Qprime  = rewards + ( ( 1. - is_done ) * self.gamma * Qprime ).detach( )
        Q       = self.critic( torch.cat( [ states,actions ], dim = 1 ) )

        critic_loss  = F.mse_loss( Q, Qprime )

        # Update Critic network
        self.critic_optimizer.zero_grad( )
        critic_loss.backward( ) 
        self.critic_optimizer.step( )

        # ====================================================== #
        # ================ Actor Optimizer Part ================ #
        # ====================================================== #
        # Actor loss, it is simply the mean of the Q function 
        # The Q function value Q( s, a ) is actually Q( s, mu( s ) ), hence the Q function is described as the parameters of mu (actor).
        # Since a is a continuous function, we can compute its gradient.   
        actor_loss = - self.critic( torch.cat( [ states, self.actor( states ) ] , dim = 1 ) ).mean( )
        
        # Update (Optimize) Actor network
        self.actor_optimizer.zero_grad( )
        actor_loss.backward( )
        self.actor_optimizer.step( )

        # The "soft" update of target networks
        for target_param, param in zip( self.actor_target.parameters( ), self.actor.parameters( ) ):
            target_param.data.copy_( param.data * self.tau + target_param.data * ( 1.0 - self.tau ) )
       
        for target_param, param in zip( self.critic_target.parameters( ), self.critic.parameters( ) ):
            target_param.data.copy_( param.data * self.tau + target_param.data * ( 1.0 - self.tau ) )


    def save( self, filename ):
        
        torch.save( self.critic.state_dict( )           , filename + "critic"              )
        torch.save( self.critic_optimizer.state_dict( ) , filename + "critic_optimizer"    )
        
        torch.save( self.actor.state_dict( )            , filename + "actor"               )
        torch.save( self.actor_optimizer.state_dict( )  , filename + "actor_optimizer"     )


    def load( self, filename ):

        # Load Critic
        self.critic.load_state_dict(            torch.load( filename + "critic"           )  )
        self.critic_optimizer.load_state_dict(  torch.load( filename + "critic_optimizer" )  )
        self.critic_target = copy.deepcopy( self.critic )

        # Load Actor
        self.actor.load_state_dict(             torch.load( filename + "actor"            )  )
        self.actor_optimizer.load_state_dict(   torch.load( filename + "actor_optimizer"  )  )
        self.actor_target = copy.deepcopy( self.actor )

class TD3:

    def __init__( self, n_state, n_action, n_hidden, act_funcs_actor, act_funcs_critic, gain = 1., gamma = 0.99, tau = 0.005, batch_size = 256, update_freq = 2 ):

        # Actor Network, its target (copy) Network, and the ADAM optimizer.
        self.actor            = NeuralNetwork( n_input = n_state, n_output = n_action, n_hidden = n_hidden, gain = gain, act_funcs = act_funcs_actor ).to( device )
        self.actor_target     = copy.deepcopy( self.actor )
        self.actor_optimizer  = optim.Adam( self.actor.parameters( ), lr = 1e-4 )

        self.critic1          = NeuralNetwork( n_state, n_action ).to( device )
        self.critic2          = NeuralNetwork( n_state, n_action ).to( device )
        self.critic_target1   = copy.deepcopy( self.critic1 )
        self.critic_target2   = copy.deepcopy( self.critic2 )
        self.critic_optimizer = torch.optim.Adam( self.critic.parameters( ), lr = 1e-3 )

        # The number of batch_size for the update 
        self.n_batch = batch_size

        # The output gain 
        self.gain = gain

        # The discount factor
        self.gamma = gamma

        # The gain for "soft" update, usually smaller than 1
        self.tau   = tau 


        # Update frequency of the policy, not the "Delayed"
        self.update_freq  = update_freq
        
        self.total_it = 0 

    def get_action( self, state ):

        # Conduct the a = mu(s), where mu is a "deterministic function"
        # Unsqueeze makes an 1 x n_s array of state. 
        state  = torch.from_numpy( state ).float( ).unsqueeze( 0 ).to( device )

        # Returns an 1 x n_a array of state
        # forward method can be omitted
        action = self.actor( state )

        # Change action from Torch to Numpy.
        # Since n_a is 1 for this case, action is simply an 1x1 array.
        # Hence, flattening the data. 
        action = action.cpu( ).data.numpy( ).flatten( )

        return action
    
    def update( self, replay_buffer, batch_size: int = 256 ):

        self.total_it += 1
        state, action, reward, next_state, is_done = replay_buffer.sample( batch_size )

        with torch.no_grad( ):
            noise       = ( torch.randn_like( action ) * self.policy_noise  ).clamp( -self.noise_clip, self.noise_clip )
            next_action = ( self.actor_target( next_state ) + noise ).clamp( -self.max_action, self.max_action )

            target_Q1, target_Q2 = self.critic_target( next_state, next_action )
            target_Q = torch.min( target_Q1, target_Q2 )
            target_Q = reward + (  ( 1. - is_done ) * self.gamma * target_Q )#.detach( )

		# Get current Q estimates
        current_Q1, current_Q2 = self.critic( state, action )

		# Compute critic loss
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

		# Optimize the critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

		# Delayed policy updates
        if self.total_it % self.policy_freq == 0:

			# Compute actor losse
            actor_loss = -self.critic.Q1( state, self.actor( state ) ).mean()
			
			# Optimize the actor 
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

			# Update the frozen target models
            for target_param, param in zip( self.critic_target.parameters( ), self.critic.parameters( ) ):
                target_param.data.copy_( self.tau * param.data + ( 1 - self.tau ) * target_param.data )
                
            for target_param, param in zip( self.actor_target.parameters( ) ,  self.actor.parameters( ) ):
                target_param.data.copy_( self.tau * param.data + ( 1 - self.tau ) * target_param.data )


    def save( self, filename ):
        
        torch.save( self.critic.state_dict( )           , filename + "critic"              )
        torch.save( self.critic_optimizer.state_dict( ) , filename + "critic_optimizer"    )
        
        torch.save( self.actor.state_dict( )            , filename + "actor"               )
        torch.save( self.actor_optimizer.state_dict( )  , filename + "actor_optimizer"     )


    def load( self, filename ):

        # Load Critic
        self.critic.load_state_dict(            torch.load( filename + "critic"           )  )
        self.critic_optimizer.load_state_dict(  torch.load( filename + "critic_optimizer" )  )
        self.critic_target = copy.deepcopy( self.critic )

        # Load Actor
        self.actor.load_state_dict(             torch.load( filename + "actor"            )  )
        self.actor_optimizer.load_state_dict(   torch.load( filename + "actor_optimizer"  )  )
        self.actor_target = copy.deepcopy( self.actor )


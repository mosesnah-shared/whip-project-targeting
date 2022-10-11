import copy
import torch
import torch.autograd
import torch.nn            as nn
import torch.nn.functional as F 
import torch.optim         as optim
import numpy               as np

from typing import Union, List

# Check whether GPU computation (i.e., CUDA) is available.
device = torch.device( "cuda" if torch.cuda.is_available( ) else "cpu" )


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


class Actor( nn.Module ):
    """
        Learning the a = mu(s) mapping, which is a deterministic function.
    """
    def __init__( self, n_state: int, n_action: int, n_hidden: int = 256, max_action: float = 1.0 ):

        # Class inheritance. 
        super( Actor, self ).__init__( )

        # Save the maximum action value 
        assert max_action >= 0
        self.max_action = max_action

        # First Layer, changes array  with size N x ( n_state  ) to N x ( n_hidden )
        self.l1 = nn.Linear(  n_state, n_hidden )

        # Second Layer, changes array with size N x ( n_hidden ) to N x ( n_hidden )
        self.l2 = nn.Linear( n_hidden, n_hidden )

        # Third Layer, changes array  with size N x ( n_hidden ) to N x ( n_action )
        self.l3 = nn.Linear( n_hidden, n_action )
        
    def forward( self, state ):
        
        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l1( state ) )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l2( x ) )

        # Applying to tanh, which ranges the value from -1 to +1
        x = torch.tanh( self.l3( x ) ) 

        # Since the x value is from -1 to +1, we change the range to -max_action to +max_action.
        return x * self.max_action

class Critic( nn.Module ):
    """
        Learning the Q(s,a) function, which is the "Quality" function. Hence, input is a concatenation of state, action and the output is a scalar. 
    """
    def __init__( self, n_state, n_action, n_hidden = 256 ):

        # Class inheritance. 
        super( Critic, self ).__init__()

        # First Layer, changes array with size N x ( n_state + n_action ) to N x ( n_hidden )
        self.l1 = nn.Linear( n_state + n_action, n_hidden )

        # Second Layer, changes array with size N x ( n_hidden ) to N x ( n_hidden )
        self.l2 = nn.Linear( n_hidden, n_hidden )

        # Third Layer, changes array with size N x ( n_hidden ) to N x ( 1 ), since Q is a scalar function. 
        self.l3 = nn.Linear( n_hidden, 1 )

    
    def forward( self, state, action ):

        # Concatenation of state and action vector.
        # The state  is assumed to be a 2D array with size N x n_s, where N is the number of samples
        # The action is assumed to be a 2D array with size N x n_a, where N is the number of samples
        # As a result of torch.cat( [ state, action ] along axis 1, ), we have size N x ( n_s + n_a ), and the dim = 0 must have the same size
        x = torch.cat( [ state, action ], dim = 1 )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l1( x ) )

        # Applying Rectified Linear Unit (ReLU) to x
        x = F.relu( self.l2( x ) )

        # A simple Ax + b combination 
        x = self.l3( x )

        # The output is a N x 1 array. 
        return x

class DDPG( object ):

    def __init__( self, n_state, n_action, max_action = 1., gamma = 0.99, tau = 0.005 ):

        # Actor Network , its target (copy) Network, and the ADAM optimizer.
        self.actor             = Actor( n_state, n_action, max_action = max_action ).to( device )
        self.actor_target      = copy.deepcopy( self.actor )
        self.actor_optimizer   = optim.Adam(  self.actor.parameters( ), lr = 1e-4 )

        # Critic Network, its target (copy) Network, and the ADAM optimizer.
        self.critic            = Critic( n_state, n_action )
        self.critic_target     = copy.deepcopy( self.critic )
        self.critic_optimizer  = optim.Adam(  self.critic.parameters( ), lr = 1e-3 )

        # The discount factor gamma and the soft-update gain tau
        self.gamma = gamma
        self.tau   = tau

        # The maximum action.
        self.max_action = max_action

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
    
    def update( self, replay_buffer, batch_size = 256 ):
        """
            Mini-batch update. 
        """
        # Randomly sample batch_size numbers of S A R S.
        states, actions, rewards, next_states, is_done = replay_buffer.sample( batch_size )

        # ====================================================== #
        # ================ Critic Optimizer Part =============== #
        # ====================================================== #
        # Keep in mind that the original paper first optimizes the Critic network

        # Critic loss 
        Qprime  = self.critic_target( next_states, self.actor_target( next_states ) )
        Qprime  = rewards + ( ( 1. - is_done ) * self.gamma * Qprime ).detach( )
        Q       = self.critic( states,actions )

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
        actor_loss = - self.critic( states, self.actor( states ) ).mean( )
        
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
        
        torch.save( self.critic.state_dict( )           , filename + "_critic"              )
        torch.save( self.critic_optimizer.state_dict( ) , filename + "_critic_optimizer"    )
        
        torch.save( self.actor.state_dict( )            , filename + "_actor"               )
        torch.save( self.actor_optimizer.state_dict( )  , filename + "_actor_optimizer"     )


    def load( self, filename ):

        # Load Critic
        self.critic.load_state_dict(            torch.load( filename + "_critic"           )  )
        self.critic_optimizer.load_state_dict(  torch.load( filename + "_critic_optimizer" )  )
        self.critic_target = copy.deepcopy( self.critic )

        # Load Actor
        self.actor.load_state_dict(             torch.load( filename + "_actor"            )  )
        self.actor_optimizer.load_state_dict(   torch.load( filename + "_actor_optimizer"  )  )
        self.actor_target = copy.deepcopy( self.actor )

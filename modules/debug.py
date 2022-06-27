"""
    A temporary script to debug the machine learnin gmodules 
"""

import sys
import gym
import time
import torch 
import numpy             as np
import matplotlib.pyplot as plt

from MLmodule import *

# Define instances of the environment, DDPG agent and the OU noise.
env   = gym.make( "Pendulum-v1" ) 

# Flags for turning on or off the render.
is_save_video = False
is_save_model = False
is_load_model = True
is_plot       = True

if is_load_model: 
    
    # Set the random seeds
    env.seed(              round( time.time( ) ) )
    env.action_space.seed( round( time.time( ) ) )

    # Get the dimension of states and actions, and also the 
    # [WARNING] This is for environments where we assume the mean of action is 0. 
    n_state    = env.observation_space.shape[ 0 ] 
    n_action   = env.action_space.shape[ 0 ]
    max_action = float( env.action_space.high  )

    agent      = DDPG( n_state, n_action, [ 256, 256 ], [ "relu", "relu", "tanh" ], [ "relu", "relu", None ], gain = float( env.action_space.high ) )        
    agent.load( "./MLmodels/" )

    # Run trial
    state = env.reset()   

    frames = []
    # The maximum trial is 200 for the pendulum 
    for _ in range( 200 ):

        if is_save_video: frames.append( env.render( mode = "rgb_array")  ) 
        env.render( )

        # Get the choice of action and the pi( a_t | s_t ) for the gradient calculation
        action = agent.get_action( state )
        new_state, _, done, _ = env.step( action )

        # If the trail encounters the terminal state
        if done: 
            break
        state = new_state

    
    env.close( )

else: # Train the model    
    
    # Set the random seeds
    seed = round( time.time( ) )
    env.action_space.seed( seed )
    torch.manual_seed(     seed )
    np.random.seed(        seed )

    # Get the dimension of states and actions, and also the 
    # [WARNING] This is for environments where we assume the mean of action is 0. 
    n_state    = env.observation_space.shape[ 0 ] 
    n_action   = env.action_space.shape[ 0 ]

    # Defining the controller
    # This case, it is a 3 x 256 x 256 x 1 neural network. 
    agent = DDPG( n_state, n_action, [ 256, 256 ], [ "relu", "relu", "tanh" ], [ "relu", "relu", None ], gain = float( env.action_space.high ) , batch_size = 256 )        

    # Define the agent, noise and replay buffers
    OUnoise       = OUNoise( n_action, env.action_space.low, env.action_space.high ) 
    replay_buffer = ReplayBuffer( n_state, n_action )

    # Saving these values to plot the performance at the end.
    frames        = [ ]
    whole_rewards = [ ]

    # For the pendulum model the best reward is 0, hence saving a -infinity value. 
    best_model_val = -np.inf

    rewards       = [ ]
    avg_rewards   = [ ]


    for episode in range( 200 ):

        # Initialize the gym environment and OU noise 
        state = env.reset( seed = round( time.time( ) ) )        
        OUnoise.reset( )

        # Initialize the episode's reward
        episode_reward = 0
        
        # For pendulum v1 gym, a single simulation is maximum 200-steps long. 
        # [REF] https://github.com/openai/gym/blob/master/gym/envs/classic_control/pendulum.py
        for step in range( 200 ):

            # Get the action value from the deterministic policy Actor network.
            action = agent.get_action( state )

            if is_save_video : frames.append( env.render( mode = 'rgb_array' ) )

            # Apply the OU noise on this action
            action = OUnoise.add_noise2action( action, step )

            # Run a single step of simulation
            new_state, reward, done, _ = env.step( action )  

            # Add this to our replay buffer, note that push simply generates the tuple and add 
            replay_buffer.add( state, action, reward, new_state, done )
            
            # Once the agent memory is full, then update the policy via replay buffer.
            if replay_buffer.current_size > agent.n_batch: agent.update( replay_buffer )        
            
            # Update the state and reward value 
            state = new_state
            episode_reward += reward

            if done:
                break

        if best_model_val <= episode_reward:
            best_model_val = episode_reward 

            # If this policy has a good result, save it 
            if is_save_model: agent.save( "./MLmodels/" ) 

        # Once a single simulation is done, append the values that will be plotted later
        rewards.append( episode_reward )
        avg_rewards.append( np.mean( rewards[ -10 : ] ) )

        sys.stdout.write("episode: {}, reward: {}, average_reward: {} \n".format( episode, np.round( episode_reward, decimals = 2 ), avg_rewards[ -1 ] ) ) 

    whole_rewards.append(  rewards  )

    env.close( )

    plt.plot( rewards     )
    plt.plot( avg_rewards )
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.show()

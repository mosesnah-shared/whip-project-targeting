"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          mujoco-py scripts for running a whip-targeting simuation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
| Creation Date:  Dec 8th,  2020
| Final Update:   Jun 20th, 2022
# ============================================================================= #

"""


import os
import sys
import scipy.io
import numpy             as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation     import Simulation
from objectives     import DistFromTip2TargetMLApproach
from constants      import my_parser
from constants      import Constants  as C
from MLmodules      import ReplayBuffer, OUNoise, DDPG
from utils          import *

is_save_model = True

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined in utils.py
parser = my_parser( )
args, unknown = parser.parse_known_args( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    args.model_name = "2D_model_w_whip_N10"
    my_sim = Simulation( args )

    # Define the DDPG controller 
    n_state  = 2 * len( my_sim.mj_data.qpos[ : ] ) 
    n_action = my_sim.n_act 

    agent         = DDPG( n_state, n_action, max_action = 30 )
    OUnoise       = OUNoise( n_action, -30 * np.ones( n_action ),  30 * np.ones( n_action ) )
    replay_buffer = ReplayBuffer( n_state, n_action )

    # The number of "batch" that will be sampled from the replay buffer will be "batch_size" 
    n_batch_size  = 256

    # For the pendulum model the best reward is 0, hence saving a -infinity value. 
    best_model_val = -np.inf

    rewards       = [ ]
    avg_rewards   = [ ]

    # Set the initial configuration of the whip downward    
    if "whip" in args.model_name: make_whip_downwards( my_sim )
    my_sim.forward( )

    obj = DistFromTip2TargetMLApproach( my_sim.mj_model, my_sim.mj_data, args, tol = 5 )
    my_sim.set_obj( obj )


    for episode in range( 1500 ):

        # Initialize the gym environment and OU noise 
        my_sim.reset()
        OUnoise.reset( )

        # Initialize the posture 
        # Choosing an arbitrary initial position
        tmp1 = np.random.uniform( -0.5*np.pi, -0.1*np.pi )
        tmp2 = np.random.uniform(  0.0*np.pi,  0.9*np.pi )
        init_pos = np.array( [ tmp1, tmp2 ] )
        init_cond = { "qpos": init_pos[ : ] ,  "qvel": np.zeros( 2 ) }
        my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )    

        # Initialize the episode's reward
        episode_reward = 0
        
        # For pendulum v1 gym, a single simulation is maximum 200-steps long. 
        # [REF] https://github.com/openai/gym/blob/master/gym/envs/classic_control/pendulum.py
        for step in range( 2100 ):

            state = my_sim.get_state(  )

            # Get the action value from the deterministic policy Actor network.
            action = agent.get_action( state )

            # Apply the OU noise on this action
            action = OUnoise.add_noise2action( action, step )

            # Run a single step of simulation
            new_state, reward = my_sim.stepML( action )  

            # Add this to our replay buffer, note that push simply generates the tuple and add 
            replay_buffer.add( state, action, reward, new_state, False )
            
            # Once the agent memory is full, then update the policy via replay buffer.
            if replay_buffer.current_size > n_batch_size: agent.update( replay_buffer, batch_size = n_batch_size )        
            
            # Update the state and reward value 
            state = new_state
            episode_reward += reward

        if best_model_val <= episode_reward:
            best_model_val = episode_reward 

            # If this policy has a good result, save it 
            if is_save_model: agent.save( "./DDPG_best_model" ) 
            
        # Once a single simulation is done, append the values that will be plotted later
        rewards.append( episode_reward )
        avg_rewards.append( np.mean( rewards[ -10 : ] ) )

        sys.stdout.write("episode: {}, reward: {}, average_reward: {} \n".format( episode, np.round( episode_reward, decimals = 2 ), avg_rewards[ -1 ] ) ) 


    scipy.io.savemat( 'DDPG_result', { "avg_rewards": avg_rewards, "rewards": rewards } )

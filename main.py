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
import argparse

import nlopt
import numpy as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
# from objectives   import DistFromTip2Target, TargetState
from constants    import Constants  as C

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Argument Parsers
parser = argparse.ArgumentParser( description = 'Parsing the arguments for running the simulation' )
parser.add_argument( '--version'     , action = 'version'     , version = C.VERSION )

parser.add_argument( '--start_time'  , action = 'store'       , type = float ,  default = 0.0,              help = 'Start time of the controller'                                                      )
parser.add_argument( '--run_time'    , action = 'store'       , type = float ,  default = 4.0,              help = 'Total run time of the simulation'                                                  )
parser.add_argument( '--model_name'  , action = 'store'       , type = str   ,  default = '2D_model' ,      help = 'Model name for the simulation'                                                     )
parser.add_argument( '--cam_pos'     , action = 'store'       , type = str   ,                              help = 'Get the whole list of the camera position'                                         )
parser.add_argument( '--mov_pars'    , action = 'store'       , type = str   ,                              help = 'Get the whole list of the movement parameters'                                     )
parser.add_argument( '--target_type' , action = 'store'       , type = int   ,                              help = 'Save data log of the simulation, with the specified frequency'                     )
parser.add_argument( '--print_mode'  , action = 'store'       , type = str   ,  default = 'normal',         help = 'Print mode, choose between [short] [normal] [verbose]'                             )
parser.add_argument( '--print_freq'  , action = 'store'       , type = int   ,  default = 60      ,         help = 'Specifying the frequency of printing the date.'                                    )
parser.add_argument( '--vid_speed'   , action = 'store'       , type = float ,  default = 1.      ,         help = 'The speed of the video. It is the gain of the original speed of the video '        )

parser.add_argument( '--record_vid'  , action = 'store_true'  ,                                             help = 'Record video of the simulation,  with the specified speed'     )
parser.add_argument( '--save_data'   , action = 'store_true'  ,                                             help = 'Save the details of the simulation'                            )
parser.add_argument( '--vid_off'     , action = 'store_true'  ,                                             help = 'Turn off the video'                                            )
parser.add_argument( '--run_opt'     , action = 'store_true'  ,                                             help = 'Run optimization of the simulation'                            )

# For jupyter compatibility.
# [REF] https://stackoverflow.com/questions/48796169/how-to-fix-ipykernel-launcher-py-error-unrecognized-arguments-in-jupyter
args, unknown = parser.parse_known_args( )

def main( ):

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    # If we use a 2D whip model
    if    "2D" and "whip" in args.model_name:

        controller_object = CartesianImpedanceController( my_sim.mjModel, my_sim.mjData, args )
        controller_object.set_ctrl_par(  mov_parameters =  [0 , -0.585 , 0.6, 0, 1.5] )
        objective = None
    

    # If we use a 3D whip model
    elif  "3D" and "whip" in args.model_name:

        ctrl = JointImpedanceController( my_sim.mj_model, my_sim.mj_data, args, is_noise = False )

        mov_pars  = np.array( [-0.9442, 1.0472,   0.0259, 1.3633, 1.7292, -1.0486,  0.0129, 1.4241, 0.5833]  )
        
        objective = DistFromTip2Target( my_sim.mjModel, my_sim.mjData, args, tol = 6 ) if "_w_" in args.model_name else None

        lb    = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )                     # Defining the bound. with np array.
        ub    = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )                     # Defining the bound. with np array.
        n_opt = 9

    else:   
        ctrl  = JointImpedanceController( my_sim.mj_model, my_sim.mj_data, args, t_start = args.start_time )
        ctrl.set_traj( mov_pars = { "q0i" : np.array( [ 1., 3. ] ), "q0f" : np.array( [ 2., 2. ] ), "D" : 1. } )
        objective = None

    my_sim.attach_ctrl( ctrl )
    my_sim.attach_objective( objective  )

    # Set the initial conditions of the simulation 
    my_sim.initialize( qpos = np.array( [ 1., 3. ] ), qvel = np.zeros( my_sim.nq )  )

    # Set the whip downward posture. 
    # [TODO] Fill in the function

    # Run the simulation
    my_sim.run( )

    # If succesful, close all the simulation. 
    my_sim.close( )


if __name__ == "__main__":

    try:  main( )
    except KeyboardInterrupt: 
        print( "\nCtrl-C was inputted. Halting the program. ", end = ' ' )




        

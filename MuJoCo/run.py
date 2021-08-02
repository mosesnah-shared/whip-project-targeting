#!/usr/bin/env python3
"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          mujoco-py scripts for running a whip-targeting simuation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
| Creation Date:  Saturday, Dec 8th, 2020
# ============================================================================= #

# ============================================================================= #
| (0A) [DESCRIPTION]
|
|  - Python Script for running mujoco-py.
|    The corresponding xml model file is under "models" directory.
|
# ============================================================================= #

# ============================================================================= #
| (0B) [KEYWORDS DEFINITION]
|       : type the following "keywords" for cases as...
|         - [BACKUP] [NAME]: Back-up code in case it's needed for the near future
|         - [TODO]: The task which should be done as soon as possible
|         - [TIP]: The reason why the following code was written.
|         - [SBA]: Acronym for "Should Be Added"  in the future. This means that the mentioned functionality should be added soon.
|         - [CBC]: Acronym for "Could Be Changed" in the future. This means that the following code could be changed (or deprecated) soon.
# ============================================================================= #

# ============================================================================= #
| (0C) [PYTHON NAMING CONVENTION]
|       Our project will follow the python naming convention, [REF]: https://stackoverflow.com/a/8423697/13437196
|       ---------------------------------------------------------------------------------------------------------
|       module_name, package_name, ClassName, method_name, ExceptionName, function_name,
|       GLOBAL_CONSTANT_NAME, global_var_name, instance_var_name, function_parameter_name, local_var_name.
# ============================================================================= #


"""



# ============================================================================= #
# (0A) [IMPORT MODULES]
# ------------------ #
# [Built-in modules] #
# ------------------ #
import sys
import os
import re
import argparse
import datetime
import shutil
import pickle

# ------------------- #
# [3rd party modules] #
# ------------------- #
import numpy       as np


import matplotlib.pyplot as plt

# import nevergrad   as ng                                                      # [BACKUP] Needed for Optimization
import sympy as sp
from sympy.utilities.lambdify import lambdify, implemented_function

# --------------- #
# [Local modules] #
# --------------- #
# See modules directory for more details
from modules.simulation   import Simulation
from modules.controllers  import ( ImpedanceController, CartesianImpedanceController, JointImpedanceController, JointSlidingController )
from modules.utils        import ( my_print, my_mkdir, args_cleanup,
                                   my_rmdir, str2float, camel2snake, snake2camel )
from modules.objectives   import DistFromTip2Target
from modules.traj_funcs   import MinJerkTrajectory
from modules.constants    import Constants

# ============================================================================= #

# ============================================================================= #
# (0B) [SYSTEM SETTINGS] + [ARGUMENT PARSER]

                                                                                # [Printing Format]
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       # Setting the numpy print options, useful for printing out data with consistent pattern.
                                                                                # precision: float precision for print/number comparison.

# [Argument parser]
# [REF] https://docs.python.org/3/library/argparse.html
parser = argparse.ArgumentParser( description = 'Parsing the arguments for running the simulation' )
parser.add_argument( '--version'    , action = 'version'     , version = Constants.VERSION )
parser.add_argument( '--run_time'   , action = 'store'       , type = float ,  default = 4.0, help = 'Total run time of the simulation'                              )
parser.add_argument( '--start_time' , action = 'store'       , type = float ,  default = 0.0, help = 'Start time of the controller'                                  )
parser.add_argument( '--model_name' , action = 'store'       , type = str   ,                 help = 'Model name for the simulation'                                 )
parser.add_argument( '--save_data'  , action = 'store'       , type = int   ,                 help = 'Save data log of the simulation, with the specified frequency' )
parser.add_argument( '--record_vid' , action = 'store'       , type = float ,                 help = 'Record video of the simulation,  with the specified speed'     )
parser.add_argument( '--cam_pos'    , action = 'store'       , type = str   ,                 help = 'Get the whole list of the camera position'                     )
parser.add_argument( '--print_mode' , action = 'store'       , default = 'normal',            help = 'Print mode, [short] [normal] [verbose]'                        )
parser.add_argument( '--vid_off'    , action = 'store_true'  ,                                help = 'Turn off the video'                                            )
parser.add_argument( '--run_opt'    , action = 'store_true'  ,                                help = 'Run optimization of the simulation'                            )


args = parser.parse_args()

args.save_dir = my_mkdir( )                                                     # Doesn't hurt to save the name directory in case

# ============================================================================= #


# ============================================================================= #

def main( ):
    # ============================================================================= #
    # (1A) [RUN SIMULATION]
    # Tossing the argument as input to contruct simulation.
    mySim = Simulation( args )

    if  "1" == args.model_name[ 0 ] and "2D" in args.model_name:

        controller_object = CartesianImpedanceController( mySim.mjModel, mySim.mjData, args )
        controller_object.set_ctrl_par(  mov_parameters =  [0 , -0.585 , 0.6, 0, 1.5] )

        # [BACKUP]
        # 1_2D_model_w_N10.xml:  mov_parameters = [ -1.40668, 0.14868, 1.46737, 0.12282, 0.81866 ] ), min_val = 0.02928
        # 1_2D_model_w_N15.xml:  mov_parameters = [ -1.39303, 0.35122, 1.56649, 0.01508, 0.79451 ] ), min_val = 0.05939
        # 1_2D_model_w_N20.xml:  mov_parameters = [ -1.56748, 0.09553, 1.57128, 0.05834, 0.80366 ] ), min_val = 0.08106
        # 1_2D_model_w_N25.xml:  mov_parameters = [ -1.3327 , 0.17022, 1.5708 , 0.13575, 0.8011  ] ), min_val = 0.02032
        objective = None
        init_cond = None

    elif "1" == args.model_name[ 0 ] and "3D" in args.model_name:

        ctrl = JointImpedanceController( mySim.mjModel, mySim.mjData, args )

        ctrl.set_ctrl_par(  K  = ( ctrl.K + np.transpose( ctrl.K ) ) / 2,
                            B  = ( ctrl.B + np.transpose( ctrl.B ) ) / 2 )

        # [TODO] [2021.07.20] [Moses C. Nah]
        # Automating the parsing? Meaning, if the mov_parameters are given simply generate the min-jerk trajectory
        # [NOTE] [2021.07.20] [Moses C. Nah]
        # Setting the trajectory is a separate member function, since we mostly modify the trajectory while keeping the gains constant.
        # [TODO] [2021.07.20] [Moses C. Nah]
        # Any modification to simply include "set_traj" and "set_ctrl_par" as a whole? Since currently, "set_ctrl_par" is only used for changing the gain.
        mov_pars  = np.array( [-1.50098, 0.     ,-0.23702, 1.41372, 1.72788, 0.     , 0.     , 0.33161, 0.95   ] )
        ctrl.traj = MinJerkTrajectory( { "pi" : mov_pars[ 0 : 4 ], "pf" : mov_pars[ 4 : 8 ], "D" : mov_pars[ -1 ] } ) # Setting the trajectory of the controller, for this case, traj = x0
        objective = DistFromTip2Target( mySim.mjModel, mySim.mjData ) if "_w_" in args.model_name else None
        init_cond = None
        # [BACKUP] [Moses C. Nah]
        # If you want to impose that the controller's K and B matrices are symmetric
        # controller_object.set_ctrl_par(  mov_parameters =  [-1.50098, 0.     ,-0.23702, 1.41372, 1.72788, 0.     , 0.     , 0.33161, 0.95   ] ,
        #                                              K  = ( controller_object.K + np.transpose( controller_object.K ) ) / 2,
        #                                              B  = ( controller_object.B + np.transpose( controller_object.B ) ) / 2 )
        # [AND THE RESULTS]
        # [Target 1] [-1.50098, 0.     ,-0.23702, 1.41372, 1.72788, 0.     , 0.     , 0.33161, 0.95   ] idx 592, output 0.05086
        # [Target 2] [-1.10279, 0.73692,-0.23271, 2.30965, 1.72788,-1.03427,-1.39626, 0.19199, 0.57881] idx 569, output 0.09177
        # [Target 3] [-0.94248, 0.81449,-1.39626, 1.72788, 2.67035,-0.69813,-1.39626, 0.05236, 0.95   ] idx 583, output 0.12684
        # [Target 4] [-0.94305, 0.     , 0.93515, 1.41372, 2.70526,-1.0472 ,-0.55688, 0.47124, 0.95   ] idx 599, output 0.01557
        # [Target 5] [-0.94248,-0.00431, 0.01293, 1.41372, 1.72788, 0.6852 ,-0.10343, 1.2896 , 0.58333] idx 357, output 0.00400

        # [CAMPOS]
        # [Target 1] camera position array [  0.40343,  0.72967, -0.71018,  3.7052 , -19.8 ,  152.2  ]
        # [Target 2] camera position array [  0.44317,  0.65556, -0.53888,  4.66869, -22.4 ,-151.8   ]
        # [Target 3] camera position array [  0.98346,  0.72647,  0.87393,  2.17427, -44.8 , -149.2  ]
        # [Target 4] camera position array [  0.98346,  0.72647,  0.87393,  2.17427, -44.8 , -149.2  ]
        # [Target 5] camera position array [  0.70927, -0.55147,  0.5744 ,  3.77981,-61.4    ,171.6  ]

    elif "2" == args.model_name[ 0 ]:

        ctrl = JointSlidingController(  mySim.mjModel, mySim.mjData, args )
        ctrl.set_ctrl_par(  Kl = 20 * np.identity( ctrl.n_act ) , Kd = 7 * np.identity( ctrl.n_act )  )

        mov_pars  = np.array( [1.72788, 0.     , 0.     , 1.41372,1.72788,-0.2, 0, 0.1, 0.12037  ])
        ctrl.traj = MinJerkTrajectory( { "pi" : mov_pars[ 0 : 4 ], "pf" : mov_pars[ 4 : 8 ], "D" : mov_pars[ -1 ] } ) # Setting the trajectory of the controller, for this case, traj = x0
        
        objective = DistFromTip2Target( mySim.mjModel, mySim.mjData ) if "_w_" in args[ 'modelName' ] else None
        init_cond = { 'qpos': np.array( [ 1.71907, 0., 0., 1.40283, 0.,-0.7, 0., 0.0069 , 0., 0.00867, 0., 0.00746, 0., 0.00527, 0., 0.00348, 0.     , 0.00286, 0.     , 0.00367, 0.     , 0.00582, 0.     , 0.00902, 0.     , 0.01283, 0.     , 0.0168 , 0.     , 0.02056, 0.     , 0.02383, 0.     , 0.02648, 0.     , 0.02845, 0.     , 0.02955, 0.     , 0.02945, 0.     , 0.02767, 0.     , 0.02385, 0.     , 0.01806, 0.     , 0.01106, 0.     , 0.00433, 0.     ,-0.00027, 0.     ,-0.00146]),
                      'qvel': np.array( [-0.07107, 0., 0.,-0.0762 , 0.,-2.92087, 0.,-0.05708, 0.,-0.10891, 0. ,-0.11822, 0.,-0.0725 , 0., 0.02682, 0.     , 0.17135, 0.     , 0.34963, 0.     , 0.54902, 0.     , 0.75647, 0.     , 0.95885, 0.     , 1.14317, 0.     , 1.29701, 0.     , 1.40942, 0.     , 1.47229, 0.     , 1.48203, 0.     , 1.44063, 0.     , 1.35522, 0.     , 1.2356 , 0.     , 1.09041, 0.     , 0.92418, 0.     , 0.73758, 0.     , 0.53229, 0.     , 0.31926, 0.     , 0.12636] ) }

        # [TEMP] [TODO] Setting the Initial condition for the optimization
        # It might be great to have a separete function to set the controller.
        # The initial condition is extracted from [REF] /Users/mosesnah/Documents/projects/whip-project-targeting/MuJoCo/results/Modularity Tasks/primitive1/data_log.txt



    else:   # If simply dummy, example model for quick debugging
        # ctrl = NullController( mySim.mjModel, mySim.mjData )
        ctrl      = None
        objective = None

    mySim.attach_ctrl( ctrl )
    mySim.attach_objective( objective  )

    if  not args.run_opt:                                                       # If simply running a single simulation without optimization

        val = mySim.run( init_cond )                                            # Getting the objective value

    else:                                 # If running nlopt optimiation

        # For optimizing the trajectory of the movement.
        if   mySim.ctrl.traj.n_pars == 5:

            lb = np.array( [ -np.pi/2,     0,     0,     0, 0.4 ] )             # Defining the bound. with np array.
            ub = np.array( [        0, np.pi, np.pi, np.pi, 1.2 ] )             # Defining the bound. with np array.

        elif mySim.ctrl.traj.n_pars == 9:

            # lb = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )                     # Defining the bound. with np array.
            # ub = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )                     # Defining the bound. with np array.

            # [TEMP] [2021.07.22]
            lb = np.array( [ -0.5 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )                     # Defining the bound. with np array.
            ub = np.array( [  1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )                     # Defining the bound. with np array.

        # Note that this is for "Minimizing the objective function"
        mySim.run_nlopt_optimization( idx = 0, input_pars = "mov_parameters", lb = lb, ub = ub, max_iter = 100 )

    # ============================================================================= #

if __name__ == "__main__":

    try:
        main( )

    except KeyboardInterrupt:

        print( "Ctrl-C was inputted. Halting the program. ", end = ' ' )
        my_rmdir( Constants.TMP_DIR )                                           # Cleaning up the tmp folder

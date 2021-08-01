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
import cv2
try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )


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
parser.add_argument( '--run_time'   , action = 'store'       , type = float,  default = 4.0, help = 'Total run time of the simulation'                              )
parser.add_argument( '--start_time' , action = 'store'       , type = float,  default = 0.0, help = 'Start time of the controller'                                  )
parser.add_argument( '--model_name' , action = 'store'       , type = str  ,                 help = 'Model name for the simulation'                                 )
parser.add_argument( '--save_data'  , action = 'store'       , default = np.nan,             help = 'Save data log of the simulation, with the specified frequency' )
parser.add_argument( '--record_vid' , action = 'store'       , default = np.nan,             help = 'Record video of the simulation,  with the specified speed'     )
parser.add_argument( '--run_opt'    , action = 'store_true'  ,                               help = 'Run optimization of the simulation'                            )
args = parser.parse_args()



# [TODO] [Moses]
# It might be beneficial, if we have some sort of "parser function", which gets the input args, and save it as the corresponding specific type.
# If video needs to be recorded or data should be saved, then append 'saveDir' element to args dictionary

args.save_dir = my_mkdir( ) if not np.isnan( args.save_data )
print( args )
exit()
args[ 'saveDir' ] = my_mkdir( ) if args[ 'recordVideo' ] or args[ 'saveData' ] or args[ 'runOptimization' ] else None


my_print( saveDir = args[ 'saveDir' ] )

# ============================================================================= #


# ============================================================================= #

def main( ):
    # ============================================================================= #
    # (1A) [GENERATE MODEL]

    model_name = args[ 'modelName' ]
    my_print( modelName = model_name )

    # ============================================================================= #

    # ============================================================================= #
    # (1C) [RUN SIMULATION]

    # [TEMP] [2021.07.22]
    VISUALIZE = False if args[ 'runOptimization' ] or args[ 'videoOFF' ] else True                    # Turn-off visualization if runOptimization


    mySim = Simulation(   model_name = model_name,
                        is_visualize = VISUALIZE,
                           arg_parse = args )


    if  "1" == args[ 'modelName' ][ 0 ] and "2D" in args[ 'modelName' ]:

        controller_object = CartesianImpedanceController( mySim.mjModel, mySim.mjData, args )
        controller_object.set_ctrl_par(  mov_parameters =  [0 , -0.585 , 0.6, 0, 1.5] )

        # [BACKUP]
        # 1_2D_model_w_N10.xml:  mov_parameters = [ -1.40668, 0.14868, 1.46737, 0.12282, 0.81866 ] ), min_val = 0.02928
        # 1_2D_model_w_N15.xml:  mov_parameters = [ -1.39303, 0.35122, 1.56649, 0.01508, 0.79451 ] ), min_val = 0.05939
        # 1_2D_model_w_N20.xml:  mov_parameters = [ -1.56748, 0.09553, 1.57128, 0.05834, 0.80366 ] ), min_val = 0.08106
        # 1_2D_model_w_N25.xml:  mov_parameters = [ -1.3327 , 0.17022, 1.5708 , 0.13575, 0.8011  ] ), min_val = 0.02032
        objective = None

    elif "1" == args[ 'modelName' ][ 0 ] and "3D" in args[ 'modelName' ]:

        ctrl = JointImpedanceController( mySim.mjModel, mySim.mjData, args )

        ctrl.set_ctrl_par(  K  = ( ctrl.K + np.transpose( ctrl.K ) ) / 2,
                            B  = ( ctrl.B + np.transpose( ctrl.B ) ) / 2 )

        # [TODO] [2021.07.20] [Moses C. Nah]
        # Automating the parsing? Meaning, if the mov_parameters are given simply generate the min-jerk trajectory
        mov_pars  = np.array( [-0.94248, 0.     , 0.     , 1.41372, 1.72788, 0.     , 0.     , 1.41372, 0.95   ] )
        ctrl.traj = MinJerkTrajectory( { "pi" : mov_pars[ 0 : 4 ], "pf" : mov_pars[ 4 : 8 ], "D" : mov_pars[ -1 ] } ) # Setting the trajectory of the controller, for this case, traj = x0
        objective = DistFromTip2Target( mySim.mjModel, mySim.mjData ) if "_w_" in args[ 'modelName' ] else None


        # [NOTE] [2021.07.20] [Moses C. Nah]
        # Setting the trajectory is a separate member function, since we mostly modify the trajectory while keeping the gains constant.
        # [TODO] [2021.07.20] [Moses C. Nah]
        # Any modification to simply include "set_traj" and "set_ctrl_par" as a whole? Since currently, "set_ctrl_par" is only used for changing the gain.

        # [BACKUP] [Moses Nah]
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


    elif "2" == args[ 'modelName' ][ 0 ]:

        ctrl = JointSlidingController(  mySim.mjModel, mySim.mjData, args )
        ctrl.set_ctrl_par(  Kl = 20 * np.identity( ctrl.n_act ) , Kd = 7 * np.identity( ctrl.n_act )  )

        # 1.72788, 0.     , 0.     , 1.41372 is the initial condition for the optimization
        mov_pars  = np.array( [1.72788, 0.     , 0.     , 1.41372,1.72788,-0.2, 0, 0.1, 0.12037  ])
        ctrl.traj = MinJerkTrajectory( { "pi" : mov_pars[ 0 : 4 ], "pf" : mov_pars[ 4 : 8 ], "D" : mov_pars[ -1 ] } ) # Setting the trajectory of the controller, for this case, traj = x0
        objective = DistFromTip2Target( mySim.mjModel, mySim.mjData ) if "_w_" in args[ 'modelName' ] else None

        # [TEMP] [TODO] Setting the Initial condition for the optimization
        # It might be great to have a separete function to set the controller.
        # The initial condition is extracted from [REF] /Users/mosesnah/Documents/projects/whip-project-targeting/MuJoCo/results/Modularity Tasks/primitive1/data_log.txt


    else:   # If simply dummy, example model for quick debugging
        # ctrl = NullController( mySim.mjModel, mySim.mjData )
        ctrl      = None
        objective = None

    mySim.attach_controller( ctrl      )
    mySim.attach_objective( objective  )

    if  not args[ 'runOptimization' ]:    # If simply running a single simulation without optimization

        val = mySim.run( )                # Getting the minimum distance between tip and target


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

    if args[ 'saveDir' ] is not None:
        mySim.save_simulation_data( args[ 'saveDir' ]  )
        shutil.copyfile( Constants.MODEL_DIR + model_name,
                         args[ 'saveDir' ] + model_name )


    mySim.reset( )

    # ============================================================================= #

if __name__ == "__main__":

    try:
        main( )

    except KeyboardInterrupt:
        print( "Ctrl-C was inputted. Halting the program. ", end = ' ' )

        if args[ 'saveDir' ] is not None:
            my_rmdir( args[ 'saveDir' ] )

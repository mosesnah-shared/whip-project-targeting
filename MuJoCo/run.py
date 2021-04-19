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

# ============================================================================= #
| (0D) [DOCOPT PARSE]
|      From now on, the written comments are specifically for "docopt" function.
|      [REF] http://docopt.org/
# ============================================================================= #

Usage:
    run.py [options]
    run.py -h | --help
    run.py -d | --debugMode

Arguments:

Options:
    -h --help                  Showing the usage and options
    --version                  Show version
    -s --saveData              Saving the neccessary data from MuJoCo simulation as a txt file in the current directory
                               [default: False]
    -r --recordVideo           Record simulation video as a .mp4 file in the current directory
                               [default: False]
    --vidRate=RATE             The rate of how fast the video runs. If 1.0, then normal speed, if 0.5, then 2 times slower.
                               [default: 1.0]
    --runTime=TIME             The total time of the simulation
                               [default: 5.0]
    --startTime=TIME           The start time of the movement, or controller
                               [default: 0.0]
    --runOptimization          Run the optimization of the simulation
                               [default: False]
    --modelName=NAME           Setting the xml model file name which will be used for the simulation.
                               The starting number of the xml model file indicates the type of simulation, hence the --modelName
                               already contains the simulation typep information.
                               List of models.
                                 - 1_2D_model_w_N10.xml    : Short  whip model                 [REF] [Moses C. Nah] [2020 BIOROB]: "Dynamic Primitives Facilitate Manipulating a Whip", [Table 2]
                                 - 1_2D_model_w_N15.xml    : Medium whip model                 [REF] [Moses C. Nah] [2020 BIOROB]: "Dynamic Primitives Facilitate Manipulating a Whip", [Table 2]
                                 - 1_2D_model_w_N20.xml    : Long   whip model                 [REF] [Moses C. Nah] [2020 BIOROB]: "Dynamic Primitives Facilitate Manipulating a Whip", [Table 2]
                                 - 1_2D_model_w_N25.xml    : Experimentally-fitted whip model  [REF] [Moses C. Nah] [2020 BIOROB]: "Dynamic Primitives Facilitate Manipulating a Whip", [Table 2]
                                 - 1_3D_model_w_N25_T1.xml : 3D w target 1                     [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", Chapter 8
                                 - 1_3D_model_w_N25_T2.xml : 3D w target 2                     [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", Chapter 8
                                 - 1_3D_model_w_N25_T3.xml : 3D w target 3                     [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", Chapter 8
                                 - 1_2D_model              : The template 2D 2-DOF upper-limb model
                                 - 1_3D_model              : The template 3D 4-DOF upper-limb model
                               [default: 1_2D_model_w_N10.xml]
    --videoOFF                 Turning off the video
                               This is useful for cases when you want to make the computation of the simulation faster
                               [default: False]
    --camPos=STRING            Setting the Camera Position of the simulation.
                               default is None
    --verbose                  Print more text
                               [default: False]
    --attachWhipModel=STRING   Auto generating the xml model file used for the simulation.
                               The input string must contain N, M, L, k and b value.
                               For more detailed information, please see myMethods.py module's generateWhipModel function.
                               default is None

Examples, try:
      python3 run.py --help
      python3 run.py --version
      python3 run.py --modelName="1_2D_model_w_N10.xml" --runTime=6
      python3 run.py --modelName="1_2D_model_w_N15.xml" --startTime=1   --runTime=6
      python3 run.py --modelName="1_2D_model_w_N15.xml" --startTime=0.1 --runTime=6 --saveData
      python3 run.py --modelName="1_2D_model_w_N20.xml" --startTime=0.1 --runTime=6 --videoOFF
      python3 run.py --modelName="1_2D_model_w_N20.xml" --startTime=0.1 --runTime=6 --videoOFF --saveData
      python3 run.py --modelName="1_2D_model_w_N10.xml" --startTime=0.1 --runTime=3 --runOptimization
      python3 run.py --modelName="1_3D_model_w_N25_T1.xml" --startTime=0.1 --runTime=3 --runOptimization
      python3 run.py --modelName="1_3D_model_w_N25_T2.xml" --startTime=0.1 --runTime=3 --recordVideo --vidRate=0.5

"""



# ============================================================================= #
# (0A) [IMPORT MODULES]
# Importing necessary modules + declaring basic configurations for running the whole mujoco simulator.

# [Built-in modules]
import sys
import os
import re
import argparse
import datetime
import shutil
import pickle

# [3rd party modules]
import numpy       as np
import cv2
try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

from docopt  import docopt

# [3rd party modules] - For Optimization
import matplotlib.pyplot as plt
# import nevergrad   as ng  # [BACKUP] Needed for Optimization

import sympy as sp
from sympy.utilities.lambdify import lambdify, implemented_function

# [Local modules]
from modules.simulation   import Simulation
from modules.controllers  import ImpedanceController, NullController
from modules.utils        import ( my_print, my_mkdir, args_cleanup,
                                   my_rmdir, str2float, camel2snake, snake2camel )
from modules.obj_funcs    import dist_from_tip2target
from modules.constants    import Constants

# ============================================================================= #

# ============================================================================= #
# (0B) [SYSTEM SETTINGS]

if sys.version_info[ : 3 ] < ( 3, 0, 0 ):                                       # Simple version check of the python version. python3+ is recommended for this file.
    my_print( NOTIFICATION = " PYTHON3+ is recommended for this script " )


                                                                                # [Printing Format]
prec = 4                                                                        # Defining the float precision for print/number comparison.
np.set_printoptions( linewidth = 8000, suppress = True, precision = prec )      # Setting the numpy print options, useful for printing out data with consistent pattern.

args = docopt( __doc__, version = Constants.VERSION )                           # Parsing the Argument
args = args_cleanup( args, '--' )                                               # Cleaning up the dictionary, discard prefix string '--' for the variables

# [TODO] [Moses]
# It might be beneficial, if we have some sort of "parser function", which gets the input args, and save it as the corresponding specific type.
# If video needs to be recorded or data should be saved, then append 'saveDir' element to args dictionary
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

    VISUALIZE = False if args[ 'runOptimization' ] or args[ 'videoOFF' ]else True                    # Turn-off visualization if runOptimization


    mySim = Simulation(   model_name = model_name,
                        is_visualize = VISUALIZE,
                           arg_parse = args )


    if   "2D" in args[ 'modelName' ]:

        controller_object = ImpedanceController( mySim.mjModel, mySim.mjData )
        controller_object.set_ctrl_par(  mov_parameters =  [-1.3327 , 0.17022, 1.5708 , 0.13575, 0.8011 ] )

        # [BACKUP]
        # 1_2D_model_w_N10.xml:  mov_parameters = [ -1.40668, 0.14868, 1.46737, 0.12282, 0.81866 ] ), min_val = 0.02928
        # 1_2D_model_w_N15.xml:  mov_parameters = [ -1.39303, 0.35122, 1.56649, 0.01508, 0.79451 ] ), min_val = 0.05939
        # 1_2D_model_w_N20.xml:  mov_parameters = [ -1.56748, 0.09553, 1.57128, 0.05834, 0.80366 ] ), min_val = 0.08106
        # 1_2D_model_w_N25.xml:  mov_parameters = [ -1.3327 , 0.17022, 1.5708 , 0.13575, 0.8011  ] ), min_val = 0.02032

    elif "3D" in args[ 'modelName' ]:

        controller_object = ImpedanceController( mySim.mjModel, mySim.mjData )
        controller_object.set_ctrl_par(  mov_parameters =  [-1.36136, 0.     ,-1.0472 , 0.47124, 0.7854 ,-1.0472 , 0.     , 0.47124, 0.58333],
                                                     K  = ( controller_object.K + np.transpose( controller_object.K ) ) / 2,
                                                     B  = ( controller_object.B + np.transpose( controller_object.B ) ) / 2 )

        obj_func = dist_from_tip2target if "_w_" in args[ 'modelName' ] else None
        # [BACKUP] [Moses Nah]
        # If you want to impose that the controller's K and B matrices are symmetric
        # controller_object.set_ctrl_par(  mov_parameters =  [-1.50098, 0.     ,-0.23702, 1.41372, 1.72788, 0.     , 0.     , 0.33161, 0.95   ] ,
        #                                              K  = ( controller_object.K + np.transpose( controller_object.K ) ) / 2,
        #                                              B  = ( controller_object.B + np.transpose( controller_object.B ) ) / 2 )
        # AND THE RESULTS
        # Target 1 [-1.50098, 0.     ,-0.23702, 1.41372, 1.72788, 0.     , 0.     , 0.33161, 0.95   ] idx 592, output 0.05086
        # Target 2 [-1.10279, 0.73692,-0.23271, 2.30965, 1.72788,-1.03427,-1.39626, 0.19199, 0.57881] idx 569, output 0.09177
        # Target 3 [-0.94248, 0.81449,-1.39626, 1.72788, 2.67035,-0.69813,-1.39626, 0.05236, 0.95   ] idx 583, output 0.12684

        # If distance is halved!
        # Target 3 [-0.94305, 0.     , 0.93515, 1.41372, 2.70526,-1.0472 ,-0.55688, 0.47124, 0.95   ] idx 599, output 0.01557
    else:   # If simply dummy, example model for quick debugging
        controller_object = NullController( mySim.mjModel, mySim.mjData )
        obj_func = None


    mySim.attach_controller( controller_object )
    mySim.attach_obj_function( obj_func  )


    if  not args[ 'runOptimization' ]:    # If simply running a single simulation without optimization
        val = mySim.run( )                # Getting the minimum distance between tip and target


    else:                                 # If running nlopt optimiation

        if   mySim.controller.n_mov_pars == 5:

            lb = np.array( [ -np.pi/2,     0,     0,     0, 0.4 ] )             # Defining the bound. with np array.
            ub = np.array( [        0, np.pi, np.pi, np.pi, 1.2 ] )             # Defining the bound. with np array.

        elif mySim.controller.n_mov_pars == 9:

            lb = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )                     # Defining the bound. with np array.
            ub = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )                     # Defining the bound. with np array.

        # Note that this is for "Minimizing the objective function"
        mySim.run_nlopt_optimization( idx = 0, input_pars = "mov_parameters", lb = lb, ub = ub, max_iter = 600 )

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

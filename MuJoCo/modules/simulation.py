"""

[PYTHON NAMING CONVENTION]
    module_name, package_name, ClassName, method_name, ExceptionName, function_name,
    GLOBAL_CONSTANT_NAME, global_var_name, instance_var_name, function_parameter_name,
    local_var_name.


"""

import sys, os
import cv2
import re
import pprint
import numpy as np
import time, datetime

import nlopt

from modules.utils        import ( my_print, quaternion2euler, camel2snake, snake2camel,
                                   MyVideo, str2float)
from modules.constants    import Constants

try:
    import mujoco_py as mjPy


except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

# from mujoco_py import

class Simulation( ):
    """
        Running a single Whip Simulation

        [INHERITANCE]

        [DESCRIPTION]


        [NOTE]
            All of the model files are saved in "models" directory, and we are using "relative directory"
            to generate and find the .xml model file. Hence do not change of "model directory" variable within this

    """

    MODEL_DIR     = Constants.MODEL_DIR
    SAVE_DIR      = Constants.SAVE_DIR
    VISUALIZE     = True

    current_time  = 0
    controller    = None                                                        # Control input function

    min_val       = np.inf                                                      # Initializing the min_val of the optimization to np.inf

    def __init__( self, model_name = None, is_visualize = True, arg_parse = None ):
        """
            Default constructor of THIS class

            [ARGUMENTS]
                [NAME]             [TYPE]        [DESCRIPTION]
                (1) model_name     string        The xml model file name for running the MuJoCo simulation.
                (2) is_visualized  boolean       Turn ON/OFF the mjViewer (visualizer) of the simulation. This flag is useful when optimizing a simulation.
                (3) arg_parse      dictionary    Dictionary which contains all the arguments given to the main `run.py` script.
        """

        if model_name is None:
            self.mjModel  = None
            self.mjSim    = None
            self.mjData   = None
            self.mjViewer = None
            self.args     = arg_parse

            my_print( WARNING = "MODEL FILE NOT GIVEN, PLEASE INPUT XML MODEL FILE WITH `attach_model` MEMBER FUNCTION" )

        else:
            # If model_name is given, then check if there exist ".xml" at the end, if not, append
            model_name = model_name + ".xml" if model_name[ -4: ] != ".xml" else model_name
            self.model_name = model_name

            # Based on the model_name, construct the simulation.
            self.mjModel  = mjPy.load_model_from_path( self.MODEL_DIR + model_name )      #  Loading xml model as and save it as "model"
            self.mjSim    = mjPy.MjSim( self.mjModel )                                    # Construct the simulation environment and save it as "sim"
            self.mjData   = self.mjSim.data                                               # Construct the basic MuJoCo data and save it as "mjData"
            self.mjViewer = mjPy.MjViewerBasic( self.mjSim ) if is_visualize else None    # Construct the basic MuJoCo viewer and save it as "myViewer"
            self.args     = arg_parse

            self.run_time    = float( self.args[ 'runTime'   ] )                 # Run time of the total simulation
            self.start_time  = float( self.args[ 'startTime' ] )                 # Start time of the movements

            # Saving the default simulation variables
            self.fps         = 60                                               # Frames per second for the mujoco render
            self.dt          = self.mjModel.opt.timestep                        # Time step of the simulation [sec]
            self.sim_step    = 0                                                # Number of steps of the simulation, in integer [-]
            self.update_rate = round( 1 / self.dt / self.fps  )                 # 1/dt = number of steps N for 1 second simulaiton, dividing this with frames-per-second (fps) gives us the frame step to be updated.
            self.g           = self.mjModel.opt.gravity                         # Calling the gravity vector of the simulation environment

            # Saving additional model parameters for multiple purposes
            self.act_names      = self.mjModel.actuator_names
            self.idx_geom_names = [ self.mjModel._geom_name2id[ name ] for name in self.mjModel.geom_names  ]



        self.VISUALIZE = is_visualize                                           # saving the VISUALIZE Flag

    def attach_model( self, model_name ):

        if self.mjModel is not None:
            my_print( WARNING = "MODEL FILE EXIST! OVERWRITTING THE WHOLE MUJOCO FILE" )

        self.__init__( model_name )

    def attach_controller( self, ctrl ):
        """
            Attaching the controller object for running the simulation.

            For detailed controller description, please check "controllers.py"

        """

        self.ctrl = ctrl

    def attach_objective( self, objective, weights = 1):
        """
            Attaching the objective function to be optimization

            The "objective" must have a form of following

            [SYNTAX]
                scalar_output = objective( mjModel, mjData )
        """

        self.objective = objective


    def run_nlopt_optimization( self, input_pars = "mov_pars", idx = 1, lb = None, ub = None, max_iter = 600 ):
        """
            Running the optimization, using the nlopt library
        """

        tmp_file = open( self.args[ 'saveDir' ] + "optimization_log.txt", "w+" )    # The txt file for saving all the iteration information

        # First check if there exist an output function
        if self.objective is None:
            raise ValueError( "Optimization cannot be executed due to the vacancy of output scalar function" )

        # Find the input parameters (input_pars) that are aimed to be optimized
        # Possible options (written in integer values) are as follows
        # [REF] https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
        idx_opt = [ nlopt.GN_DIRECT_L, nlopt.GN_DIRECT_L_RAND, nlopt.GN_DIRECT, nlopt.GN_CRS2_LM, nlopt.GN_ESCH  ]
        self.algorithm = idx_opt[ idx ]                                             # Selecting the algorithm to be executed


        # self.n_opt = self.ctrl.traj.n_pars                                      # Getting the dimension of the input vector, i.e.,  the number of parameters that are aimed to be optimized
        self.n_opt = 5
        self.opt   = nlopt.opt( self.algorithm, self.n_opt )                    # Defining the class for optimization

        self.opt.set_lower_bounds( lb )                                         # Setting the upper/lower bound of the optimization
        self.opt.set_upper_bounds( ub )                                         # Setting the upper/lower bound of the optimization
        self.opt.set_maxeval( max_iter )                                        # Running 600 (N) iterations for the simulation.

        init = ( lb + ub ) * 0.5 + 0.05 * lb                                    # Setting an arbitrary non-zero initial step

        self.opt.set_initial_step( init )


        def nlopt_objective( pars, grad ):                                       # Defining the objective function that we are aimed to optimize.

            # pars for this case is the number of movement parameters
            self.ctrl.traj.set_traj(  { "pi" : np.array( [1.72788, 0.     , 0.     , 1.41372] ), "pf" : pars[ 0 : 4 ], "D" : pars[ -1 ] }   )
            val = self.run( )                                                   # Running a single simulation and get the minimum distance achieved

            self.reset( )

            my_print( Iter = self.opt.get_numevals( ) + 1, inputPars = pars, output = val )                     # Printing the values onto the screen
            my_print( Iter = self.opt.get_numevals( ) + 1, inputPars = pars, output = val, file = tmp_file )    # Printing the values onto the txt file

            return val


        self.opt.set_min_objective( nlopt_objective )
        self.opt.set_stopval( 1e-8 )                                               # If value is 0 then target is hit!

        self.xopt = self.opt.optimize( ( lb + ub ) * 0.5 )                      # Start at the mid-point of the lower and upper bound

        my_print(  optimalInput = self.xopt[ : ],
                  optimalOutput = self.opt.last_optimum_value( ) )              # At end, printing out the optimal values and its corresponding movement parameters

        tmp_file.close()

    def set_cam_pos( self, cam_pos ):
        tmp = str2float( self.args[ 'camPos' ] )
        self.mjViewer.cam.lookat[ 0:3 ] = tmp[ 0 : 3 ]
        self.mjViewer.cam.distance      = tmp[ 3 ]
        self.mjViewer.cam.elevation     = tmp[ 4 ]
        self.mjViewer.cam.azimuth       = tmp[ 5 ]

    def run( self ):
        """
            Description
            -----------
            Running a single simulation.

            Arguments
            ---------
                [VAR NAME]             [TYPE]     [DESCRIPTION]
                (1) run_time           float      The whole run time of the simulation.
                (2) ctrl_start_time    float
        """

        # Check if mjModel or mjSim is empty and raise error
        if self.mjModel is None or self.mjSim is None:
            raise ValueError( "mjModel and mjSim is Empty! Add it before running simulation"  )

        # Warn the user if input and output function is empty
        if self.ctrl is None:
            raise ValueError( "CONTROLLER NOT ATTACHED TO SIMULATION. \
                               PLEASE REFER TO METHOD 'attach_objectivetion' and 'attach_controller' " )


        if self.args[ 'recordVideo' ]:
            vid = MyVideo( fps = self.fps * float( self.args[ 'vidRate' ] ),
                       vid_dir = self.args[ 'saveDir' ] )                       # If args doesn't have saveDir attribute, save vid_dir as None

        if self.args[ 'saveData' ] or self.args[ 'runOptimization' ]:
            file = open( self.args[ 'saveDir' ] + "data_log.txt", "a+" )

        # Setting the initial condition of the robot.
        self.set_initial_condition()

        if self.args[ 'camPos' ] is not None:
            self.set_cam_pos( self.args[ 'camPos' ] )

        # [TEMP] [2021.07.22] -1.86358
        self.mjData.qpos[ : ] = np.array( [ 1.71907, 0.     , 0.     , 1.40283, 0.     ,-0.7, 0.     , 0.0069 , 0.     , 0.00867, 0.     , 0.00746, 0.     , 0.00527, 0.     , 0.00348, 0.     , 0.00286, 0.     , 0.00367, 0.     , 0.00582, 0.     , 0.00902, 0.     , 0.01283, 0.     , 0.0168 , 0.     , 0.02056, 0.     , 0.02383, 0.     , 0.02648, 0.     , 0.02845, 0.     , 0.02955, 0.     , 0.02945, 0.     , 0.02767, 0.     , 0.02385, 0.     , 0.01806, 0.     , 0.01106, 0.     , 0.00433, 0.     ,-0.00027, 0.     ,-0.00146])
        self.mjData.qvel[ : ] = np.array( [-0.07107, 0.     , 0.     ,-0.0762 , 0.     ,-2.92087, 0.     ,-0.05708, 0.     ,-0.10891, 0.     ,-0.11822, 0.     ,-0.0725 , 0.     , 0.02682, 0.     , 0.17135, 0.     , 0.34963, 0.     , 0.54902, 0.     , 0.75647, 0.     , 0.95885, 0.     , 1.14317, 0.     , 1.29701, 0.     , 1.40942, 0.     , 1.47229, 0.     , 1.48203, 0.     , 1.44063, 0.     , 1.35522, 0.     , 1.2356 , 0.     , 1.09041, 0.     , 0.92418, 0.     , 0.73758, 0.     , 0.53229, 0.     , 0.31926, 0.     , 0.12636] )
        self.mjSim.forward()       # Update

        while self.current_time <= self.run_time:

            if self.sim_step % self.update_rate == 0:

                if self.mjViewer is not None:
                    self.mjViewer.render( )                                     # Render the simulation

                if not self.args[ 'runOptimization' ]:
                    my_print( currentTime = self.current_time  )

                if self.args[ 'verbose' ]:
                    my_print( camParameters = [ self.mjViewer.cam.lookat[ 0 ], self.mjViewer.cam.lookat[ 1 ], self.mjViewer.cam.lookat[ 2 ],
                                                self.mjViewer.cam.distance,    self.mjViewer.cam.elevation,   self.mjViewer.cam.azimuth ] )

                if self.args[ 'recordVideo' ]:
                    vid.write( self.mjViewer )

                if self.args[ 'saveData' ]:
                    my_print(  currentTime       = self.current_time, file              = file   )

            # [input controller]
            # input_ref: The data array that are aimed to be inputted (e.g., qpos, qvel, qctrl etc.)
            # input_idx: The specific index of input_ref data array that should be inputted
            # input:     The actual input value which is inputted to input_ref

            if self.start_time >= self.current_time:
                input_ref, input_idx, input = self.ctrl.input_calc( 0 )
            else:
                input_ref, input_idx, input = self.ctrl.input_calc( self.current_time - self.start_time )

            if input_ref is not None:
                input_ref[ input_idx ] = input


            self.mjSim.step( )                                                  # Single step update

            if( self.is_sim_unstable() ):                                       # Check if simulation is stable

                # If not optimization, and result unstable, then save the detailed data
                print( "[WARNING] UNSTABLE SIMULATION, HALTED AT {0:f} for at {1:f}".format( self.current_time, self.run_time )  )
                if self.args[ 'saveData' ] or self.args[ 'runOptimization' ]:
                    file.close( )

                break

            self.current_time = self.mjData.time                                # Update the current_time variable of the simulation


            # Calculate objective value based on mujoco Data
            if self.objective is not None:
                self.min_val = min( self.min_val, self.objective.output_calc( )  )

            if self.sim_step % self.update_rate == 0:

                if self.args[ 'saveData' ]:
                    # Saving all the necessary datas for the simulation
                    # my_print(  inputVal = input,
                    #             minVal  = self.min_val,
                    #                ZFT  = self.ctrl.phi,
                    #                dZFT = self.ctrl.dphi,
                    #                file = file )

                    my_print(   inputVal = input,
                                    qPos = self.mjData.qpos[ : ],
                                    qVel = self.mjData.qvel[ : ],
                                      qd = self.ctrl.qd[ : ],
                                      s  = self.ctrl.s[ : ],
                        geomXYZPositions = self.mjData.geom_xpos[  self.idx_geom_names ],
                                    dist = self.objective.output_calc( ) ,
                                    file = file         )


                else:
                    pass


            self.sim_step += 1

        if self.args[ 'recordVideo' ]:
            vid.release( )                                                      # If simulation is finished, wrap-up the video file.

        if self.args[ 'saveData' ]:
            file.close()


        return self.min_val                                                     # Returning the minimum value achieved with the defined objective function

    def set_initial_condition( self ):
        """
            Manually setting the initial condition of the system.
        """

        ctrl_name = self.ctrl.__class__.__name__.lower()                        # Getting the name of the controller. The controller names are indicated in "input_ctrls.py"

        nJ = self.ctrl.n_act                                                    # Getting the number of active joints


        if nJ != 0 and "2D" not in self.model_name:
            self.mjData.qpos[ 0 : nJ ] = self.ctrl.traj.pars[ "pi" ]            # Setting the initial posture of the upper-limb as the movement parameters
            self.mjSim.forward()                                                # Update Needed for setting the posture of the upper limb by "forward" method.


        # The whip should face downward, complying to gravity at rest.
        # Hence, manually setting the whip to do so.
        if "_w_" in self.model_name:                                            # If whip is attached to the model.
                                                                                # This is distinguished by the xml model file's name. "_w_" is included on the name.

            tmp = self.mjData.get_body_xquat( "body_node1" )                         # Getting the quaternion angle of the whip handle
            yaw, pitch, roll = quaternion2euler( tmp )

            if   nJ == 2: # for 2DOF Robot - The connection between the whip and upper-limb has 1DOF
                self.mjData.qpos[ nJ ] = + pitch if round( roll ) == 0 else np.pi - pitch

            elif nJ == 4: # for 4DOF Robot - The connection between the whip and upper-limb has 2DOF
                self.mjData.qpos[ nJ     ] = - roll                             # Setting the handle posture to make the whip being straight down at equilibrium.
                self.mjData.qpos[ nJ + 1 ] = + pitch                            # Setting the handle posture to make the whip being straight down at equilibrium.


            self.mjSim.forward()                                                # Again, updating the posture after setting the "qpos" value.


        else:
            NotImplementedError( )

    def get_output_array( self ):

        if self.output_array is not None:
            return self.output_array.copy()

        else:
            return None


    def save_simulation_data( self, dir ):
        """
            Save all the details of the controller parameters, inputs and output of the simulation
        """

        if dir is not None and dir[ -1 ] != "/":                                # Quick Check of whether result_dir has backslash "/" at the end
            dir += "/"                                                          # Append the backslash

        # [TIP] [MOSES]
        # By using the "with" function you don't need to call f.close( ), the file will automatically close the opened file.
        # [REF] https://lerner.co.il/2015/01/18/dont-use-python-close-files-answer-depends/

        with open( dir + "simulation_details.txt", "w+" ) as f:
            pprint.pprint( self.ctrl.__dict__, f )                        # Using pretty-print (pprint) to flush out the data in a much readable format
            print( self.args                , file = f )                        # Flushing out all the arguments detail.


    def is_sim_unstable( self ):
        """
            Description
            -----------
            Check whether the simulation is stable.
            If the simulation exceeds some threshold value (10^6) for this case, then halting the simulation

        """
        thres = 1 * 10 ** 6

        if ( max( np.absolute( self.mjData.qpos ) ) > thres ) or \
           ( max( np.absolute( self.mjData.qvel ) ) > thres ) or \
           ( max( np.absolute( self.mjData.qacc ) ) > thres ):
           return True

        else:
           return False


    def reset( self ):
        """
            Reseting the mujoco simulation
        """
        self.current_time = 0
        self.sim_step     = 0
        self.min_val      = np.inf
        self.mjSim.reset( )

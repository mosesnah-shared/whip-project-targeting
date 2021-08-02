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
import shutil
import numpy as np
import time, datetime
import nlopt
# import glfw

from modules.utils        import ( my_print, quaternion2euler, camel2snake, snake2camel,
                                   MyVideo, str2float, my_mvdir, my_rmdir )
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
    TMP_DIR       = Constants.TMP_DIR

    def __init__( self, args  ):

        # Saving the boolean simulation variables
        self.is_save_data = True  if args.save_data                    else False
        self.is_vid_rec   = True  if args.record_vid                   else False
        self.is_vid_on    = False if args.run_opt or args.vid_off      else True
        self.is_run_opt   = args.run_opt

        # Based on the model_name, construct the simulation.
        self.model_name = args.model_name + ".xml" if args.model_name[ -4: ] != ".xml" else args.model_name
        self.mjModel    = mjPy.load_model_from_path( self.MODEL_DIR + self.model_name ) #  Loading xml model as and save it as "model"
        self.mjSim      = mjPy.MjSim( self.mjModel )                                    # Construct the simulation environment and save it as "sim"
        self.mjData     = self.mjSim.data                                               # Construct the basic MuJoCo data and save it as "mjData"
        self.mjViewer   = mjPy.MjViewerBasic( self.mjSim ) if self.is_vid_on else None  # Construct the basic MuJoCo viewer and save it as "myViewer"
        self.ctrl       = None
        self.args       = args
        self.obj_val    = np.inf

        # Saving the default simulation variables
        self.fps          = 60                                                  # Frames per second for the mujoco render
        self.dt           = self.mjModel.opt.timestep                           # Time step of the simulation [sec]
        self.run_time     = args.run_time                                       # Total run time (tr) of simulation
        self.start_time   = args.start_time                                     # Start of controller
        self.step         = 0                                                   # Number of steps of the simulation, in integer [-]
        self.t            = 0
        self.g            = self.mjModel.opt.gravity                            # Calling the gravity vector of the simulation environment

        # Steps for updating the record or data save
        self.vid_step     = round( 1 / self.dt / self.fps          )            # The very basic update rate.

        # If record is on, then calculate rec_step
        if self.is_vid_rec:
            self.rec_step     = round( self.vid_step * args.record_vid )        # vid_rec is given as 0.2, 0.1, 0.3x etc.

        # If save_data is on, then calculate save_step
        if self.is_save_data:
            self.save_step    = round( 1 / self.dt / args.save_data    )        # save_data is given as 60Hz, 30Hz, etc.


    def attach_ctrl( self, ctrl ):
        """For detailed controller description, please check 'controllers.py' """
        self.ctrl = ctrl

    def attach_objective( self, objective, weights = 1 ):
        """ Appending objective function with weights as coefficients, refer to 'objectives.py for details' """
        self.objective = objective

    def run( self, init_cond = None ):
        """ Running the simulation """

        self._init_sim(  )
        self._set_init_cond( init_cond )

        while self.t <= self.run_time:

            # Render the simulation
            if self.step % self.vid_step == 0:
                if self.mjViewer is not None:
                    self.mjViewer.render( )

            # [NOTE] [2021.08.01] [Moses C. Nah]
            # To use this, you need to modify "mjviewer.py" file under "mujoco_py"
            if self.mjViewer.paused:
                continue

            # [Calculate Input]
            # input_ref: The data array that are aimed to be inputted (e.g., qpos, qvel, qctrl etc.)
            # input_idx: The specific index of input_ref data array that should be inputted
            # input:     The actual input value which is inputted to input_ref
            if self.start_time >= self.t:
                input_ref, input_idx, input = self.ctrl.input_calc( 0 )
            else:
                input_ref, input_idx, input = self.ctrl.input_calc( self.t - self.start_time )

            # Setting this on the controller
            if input_ref is not None:
                input_ref[ input_idx ] = input


            # [Calculate objective Value]
            if self.objective is not None:
                self.obj_val = min( self.obj_val, self.objective.output_calc( )  )

            if self.is_vid_rec and self.step % self.rec_step == 0 :
                self.vid.write( self.mjViewer )

            # [Printing out the details]
            # [TODO] [Moses C. Nah] [2021.08.01] Making this code much cleaner
            if self.step % self.vid_step == 0:
                if not self.args.run_opt and self.args.print_mode == "normal" :
                    my_print( currentTime = self.t  )

                if self.args.print_mode == "verbose":
                    my_print( cameraPositions = [ self.mjViewer.cam.lookat[ 0 ], self.mjViewer.cam.lookat[ 1 ], self.mjViewer.cam.lookat[ 2 ],
                                                  self.mjViewer.cam.distance,    self.mjViewer.cam.elevation,   self.mjViewer.cam.azimuth ] )

            if self.is_save_data and self.step % self.save_step == 0:
                my_print( currentTime = self.t,
                             inputVal = input,
                                 qPos = self.mjData.qpos[ : ],
                                 qVel = self.mjData.qvel[ : ], file = self.file )
                     #               qd = self.ctrl.qd[ : ],
                     #               s  = self.ctrl.s[ : ],
                     # geomXYZPositions = self.mjData.geom_xpos[ self.idx_geom_names ],
                     #             dist = self.objective.output_calc( ) ,
                     #           objVal = self.obj_val,
                     #             file = file         )


            self.mjSim.step( )
            self.t = self.mjData.time                                           # Update the current time variable of the simulation
            self.step += 1

            if( self._is_sim_unstable() ):                                      # Check if simulation is stable

                # If not optimization, and result unstable, then save the detailed data
                print( "[WARNING] UNSTABLE SIMULATION, HALTED AT {0:f} for at {1:f}".format( self.t, self.run_time )  )
                break

        self._close( )

        return self.obj_val                                                     # Returning the minimum value achieved with the defined objective function


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


    # ======================================================================== #
    # INTERNAL FUNCTIONS
    # ======================================================================== #
    def _set_init_cond( self, init_cond = None ):
        """ Manually setting the initial condition of the system. """

        if init_cond is None:

            nJ = self.ctrl.n_act                                                # Getting the number of active joints

            self.mjData.qpos[ 0 : nJ ] = self.ctrl.traj.pars[ "pi" ]            # Setting the initial posture of the upper-limb as the movement parameters
            self.mjSim.forward()                                                # Update Needed for setting the posture of the upper limb by "forward" method.

            # The whip should face downward, complying to gravity at rest.
            # Hence, manually setting the whip to do so.
            if "_w_" in self.model_name:                                        # If whip is attached to the model.
                                                                                # This is distinguished by the xml model file's name. "_w_" is included on the name.
                self._set_whip_downward(  )

        else:
            self.mjData.qpos = init_cond[ "qpos" ]
            self.mjData.qvel = init_cond[ "qvel" ]
            self.mjSim.forward()                                                # Again, updating the posture after setting the "qpos" value.


    def _init_sim( self ):

        # Warn the user if input and output function is empty
        if self.ctrl is None:
            raise ValueError( "CONTROLLER NOT ATTACHED TO SIMULATION. Please attach Ctrl object via 'attach_ctrl' " )

        if self.is_vid_rec:
            print( "HO", self.fps * self.args.record_vid )
            self.vid = MyVideo( fps = self.fps * self.args.record_vid ,
                            vid_dir = self.args.save_dir )

        if self.is_save_data:
            self.file = open( self.args.save_dir + "data_log.txt", "a+" )

        if self.args.cam_pos is not None:
            self._set_cam_pos( self.args.cam_pos )

    def _set_cam_pos( self, cam_pos ):
        tmp = str2float( self.args.cam_pos )
        self.mjViewer.cam.lookat[ 0:3 ] = tmp[ 0 : 3 ]
        self.mjViewer.cam.distance      = tmp[ 3 ]
        self.mjViewer.cam.elevation     = tmp[ 4 ]
        self.mjViewer.cam.azimuth       = tmp[ 5 ]

    def _save_sim_details( self  ):

        with open( self.args.save_dir + "sim_details.txt", "w+" ) as f:
            pprint.pprint( self.ctrl.__dict__, f )                              # Using pretty-print (pprint) to flush out the data in a much readable format
            print( self.args    , file = f )                                    # Flushing out all the arguments detail.


    def _is_sim_unstable( self ):
        """ Check whether the simulation is stable. If the acceleration exceeds some threshold value, then halting the simulation """
        return True if max( np.absolute( self.mjData.qacc ) ) > 1 * 10 ** 6 else False

    def _set_whip_downward( self  ):
        """ Setting the whip posture downward at rest """

        tmp = self.mjData.get_body_xquat( "body_node1" )                # Getting the quaternion angle of the whip handle
        yaw, pitch, roll = quaternion2euler( tmp )

        nJ = self.ctrl.n_act

        if   nJ == 2: # for 2DOF Robot - The connection between the whip and upper-limb has 1DOF
            self.mjData.qpos[ nJ ] = + pitch if round( roll ) == 0 else np.pi - pitch

        elif nJ == 4: # for 4DOF Robot - The connection between the whip and upper-limb has 2DOF
            self.mjData.qpos[ nJ     ] = - roll                         # Setting the handle posture to make the whip being straight down at equilibrium.
            self.mjData.qpos[ nJ + 1 ] = + pitch                        # Setting the handle posture to make the whip being straight down at equilibrium.

        self.mjSim.forward()


    def _close( self ):
        """ Wrapping up the simulation"""
        if self.is_vid_rec:
            self.vid.release( )

        if self.is_save_data:
            self.file.close(  )

        if self.is_vid_rec or self.is_save_data or self.is_run_opt:
            self._save_sim_details(  )
            shutil.copyfile( Constants.MODEL_DIR + self.args.model_name,
                              self.args.save_dir + self.args.model_name )

            my_mvdir( self.args.save_dir, self.SAVE_DIR  )                      # Move to the save reposito

        my_rmdir( Constants.TMP_DIR )                                           # Cleaning up the tmp folder

        self._reset( )

    def _reset( self ):
        """ Reseting the mujoco simulation without changing the model/controller etc. """
        self.t            = 0
        self.sim_step     = 0
        self.obj_val      = np.inf
        self.mjSim.reset( )

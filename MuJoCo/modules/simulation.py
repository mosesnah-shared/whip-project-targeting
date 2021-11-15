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
        self.init_cond    = None

        # Steps for updating the record or data save
        self.vid_step         = round( 1 / self.dt / self.fps          )            # The very basic update rate.

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

    def wait_until( self, time ):
        """
            Simply waiting the simulation. This is used BEFORE the controller is activated, i.e., when t <= start_time.
            This function is handy when we want to wait for the model's residual dynamics to be stopped.
            [INPUT]
                [VAR NAME]             [TYPE]     [DESCRIPTION]
                (1) time               float      The time of how long the simulation will just not run
        """
        while self.mjData.time < time:

            input_ref, input_idx, input = self.ctrl.input_calc( 0  )
            input_ref[ input_idx ]      = input
            self.mjSim.step( )                                                  # Single step update

        self.run_time   += time
        self.start_time += time
        self.t = self.mjData.time

    def run( self, init_cond = None ):
        """ Running the simulation """

        self._init_sim(  )
        self._set_init_cond( init_cond )

        # self.wait_until( 100 )

        while self.t <= self.run_time:

            # Render the simulation
            if self.step % self.vid_step == 0:
                if self.mjViewer is not None:
                    self.mjViewer.render( )

            # [NOTE] [2021.08.01] [Moses C. Nah]
            # To use this, you need to modify "mjviewer.py" file under "mujoco_py"

            if self.mjViewer is not None and self.mjViewer.is_reset:
                self._back_to_init( )
                self.mjViewer.is_reset = False
                continue

            if self.mjViewer is not None and self.mjViewer.is_paused:
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
                self.obj_val = min( self.obj_val, self.objective.output_calc()  )

            if self.is_vid_rec and self.step % self.rec_step == 0 :
                self.vid.write( self.mjViewer )

            # [Printing out the details]
            # [TODO] [Moses C. Nah] [2021.08.01] Making this code much cleaner
            # if self.step % self.vid_step == 0:
            #     if not self.args.run_opt and self.args.print_mode == "normal" :
            #         my_print( currentTime = self.t, output = self.obj_val  )
            #
            if self.args.print_mode == "verbose":
                my_print( cameraPositions = [ self.mjViewer.cam.lookat[ 0 ], self.mjViewer.cam.lookat[ 1 ], self.mjViewer.cam.lookat[ 2 ],
                                              self.mjViewer.cam.distance,    self.mjViewer.cam.elevation,   self.mjViewer.cam.azimuth ] )

            if self.is_save_data and self.step % self.save_step == 0:
                my_print( currentTime = self.t,
                                 qPos = self.mjData.qpos[ : ],
                                 qVel = self.mjData.qvel[ : ],
                                qPos0 = self.ctrl.x0,
                                qVel0 = self.ctrl.dx0,
                     geomXYZPositions = self.mjData.geom_xpos[ self.ctrl.idx_geom_names ],
                               taus   = self.ctrl.tau, file = self.file   )


            self.mjSim.step( )
            self.t = self.mjData.time                                           # Update the current time variable of the simulation
            self.step += 1


            if( self._is_sim_unstable() ):                                      # Check if simulation is stable

                # If not optimization, and result unstable, then save the detailed data
                print( "[WARNING] UNSTABLE SIMULATION, HALTED AT {0:f} for at {1:f}".format( self.t, self.run_time )  )
                break

        return self.obj_val                                                     # Returning the minimum value achieved with the defined objective function

    def reset( self ):
        """ Reseting the mujoco simulation without changing the model/controller etc. """
        self.t            = 0
        self.sim_step     = 0
        self.obj_val      = np.inf
        self.mjSim.reset( )

    def close( self ):
        """ Wrapping up the simulation"""
        if self.is_vid_rec:
            self.vid.release( )

        if self.is_save_data:
            self.file.close(  )

        if self.is_vid_rec or self.is_save_data or self.is_run_opt:
            self._save_sim_details(  )
            shutil.copyfile( Constants.MODEL_DIR + self.args.model_name,
                              self.args.save_dir + self.args.model_name )

            my_mvdir( self.args.save_dir, self.SAVE_DIR  )
            my_rmdir( Constants.TMP_DIR )



    # ======================================================================== #
    # INTERNAL METHODS
    # ======================================================================== #
    def _set_init_cond( self, init_cond = None ):
        """ Manually setting the initial condition of the system. """

        self.init_cond = init_cond

        if self.init_cond is None:

            nJ = self.ctrl.n_act                                                # Getting the number of active joints

            self.mjData.qpos[ 0 : nJ ] = self.ctrl.traj.pars[ "pi" ]            # Setting the initial posture of the upper-limb as the movement parameters
            self.mjSim.forward()                                                # Update Needed for setting the posture of the upper limb by "forward" method.

            # The whip should face downward, complying to gravity at rest.
            # Hence, manually setting the whip to do so.
            if "_w_" in self.model_name:                                        # If whip is attached to the model.
                                                                                # This is distinguished by the xml model file's name. "_w_" is included on the name.
                self._set_whip_downward(  )

        else:
            self.mjData.qpos[ : ] = self.init_cond[ "qpos" ]
            self.mjData.qvel[ : ] = self.init_cond[ "qvel" ]
            self.mjSim.forward()                                                # Again, updating the posture after setting the "qpos" value.

    # [TODO] [Moses C. Nah] [2021.08.03]
    # Just simply merging this function to reset
    def _back_to_init( self ):
        """ Going back to the initial condition of the simulation"""
        self._set_init_cond( self.init_cond )
        self.t            = 0
        self.sim_step     = 0
        self.obj_val      = np.inf

    def _init_sim( self ):

        # Warn the user if input and output function is empty
        if self.ctrl is None:
            raise ValueError( "CONTROLLER NOT ATTACHED TO SIMULATION. Please attach Ctrl object via 'attach_ctrl' " )

        if self.is_vid_rec:
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
            print( self           , file = f )
            print( self.ctrl      , file = f )
            print( self.args      , file = f )



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

    # ======================================================================== #
    # MAGIC METHODS
    # ======================================================================== #
    def __str__( self ):
        """ Starting and ending with __ are called "magic methods" [REF] https://www.tutorialsteacher.com/python/magic-methods-in-python """
        return str( vars( self ) )

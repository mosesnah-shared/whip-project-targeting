import os
import sys
import shutil
import numpy           as np
import mujoco_py       as mjPy
import moviepy.editor  as mpy


from utils     import *
from datetime  import datetime
from constants import Constants as C

epsilon = sys.float_info.epsilon


class Simulation:
    """
        Running a single Whip Simulation
    """

    def __init__( self, args  ):

        # The whole argument passed to the main python file. 
        self.args = args

        # Check if we save data, record video, or video turn off the render.  
        self.is_save_data   = args.save_data
        self.is_record_vid  = args.record_vid
        self.is_vid_off     = args.vid_off


        # Controller, objective function and the objective values' array
        self.ctrl     = None
        self.obj      = None
        self.obj_arr  = None

        # Save the model name.
        self.model_name = args.model_name

        # Construct the basic mujoco attributes
        self.mj_model  = mjPy.load_model_from_path( C.MODEL_DIR + self.model_name + ".xml" ) 
        self.mj_sim    = mjPy.MjSim( self.mj_model )    
        self.mj_data   = self.mj_sim.data                                              
        self.mj_viewer = mjPy.MjViewerBasic( self.mj_sim ) if not self.is_vid_off else None  

        # Save the number of generalized coordinate of the system 
        self.nq = len( self.mj_data.qpos ) 
        
        # We set the normal frames-per-second as 60. 
        self.fps = 60                   

        # Time-step (dt), start time of the controller (ts) and total runtime (T) of the simulation
        self.dt  = self.mj_model.opt.timestep   
        self.ts  = self.args.start_time                                              
        self.T   = self.args.run_time                     

        # If it is 60 frames per second, then for vid_speed = 2, we save 30 frames per second, hence 
        # The number of steps for a single-second is ( 1. / self.dt ), and divide 
        # [Example] for dt = 0.001, we have 1000 for ( 1. / self.dt ), and for vid_speed = 2, self.fps / self.vid_speed = 30 
        #           Hence, we save/render the video every round( 1000 / 30 ) time steps. 
        self.vid_step   = round( ( 1. / self.dt ) / ( self.fps / self.args.vid_speed )  )

        # Step for printing the data. 
        self.print_step = round( ( 1. / self.dt ) / self.args.print_freq  )                

        # Generate tmp directory before moving to results folder 
        self.tmp_dir  = C.TMP_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S/" )
        os.mkdir( self.tmp_dir )  

        if   self.is_save_data: self.save_file = open( self.tmp_dir + "sim_data.txt", "a+" )
        
        # This variable is for saving the video of the simulation
        self.frames = [ ]

    def initialize( self, qpos: np.ndarray, qvel: np.ndarray ):
        """
            Initialize the Simulation. 
            Note that the controller and objective function is NOT erased. 
            Meaning, initialize must be called "AFTER" self.set_ctrl and self.set_obj
        """

        # Current time (t) of the simulation 
        self.t  = 0
          
        # Number of steps of the simulation. 
        self.n_steps = 0  

        # Save initial q_pos and q_vel 
        self.init_qpos = qpos 
        self.init_qvel = qvel 

        # If the array is shorter than the actual self.nq in the model, just fill it with zero 
        self.mj_data.qpos[ : ] = qpos[ : self.nq ] if len( qpos ) >= self.nq else np.concatenate( ( qpos, np.zeros( self.nq - len( qpos ) ) ) , axis = None )
        self.mj_data.qvel[ : ] = qvel[ : self.nq ] if len( qvel ) >= self.nq else np.concatenate( ( qvel, np.zeros( self.nq - len( qvel ) ) ) , axis = None )

        # Forward the simulation to update the posture 
        self.mj_sim.forward( )


    def reset( self ):
        """
            Reset the simulation. 
        """
        self.mj_sim.reset( )
        self.initialize( qpos = self.init_qpos, qvel = self.init_qvel )
        if "whip" in self.args.model_name: make_whip_downwards( self )

        self.obj_arr = np.zeros( round( self.T / self.dt )  )
        
    def close( self ):
        """ 
            Wrapping up the simulation
        """
        # If video should be recorded, write the video file. 
        if self.is_record_vid and self.frames is not None:
            clip = mpy.ImageSequenceClip( self.frames, fps = self.fps )
            clip.write_videofile( self.tmp_dir + "video.mp4", fps = self.fps, logger = None )

        # If video recorded/save data is true, then copy the model, main file and the arguments passed
        if self.is_record_vid or self.is_save_data:
            shutil.copyfile( C.MODEL_DIR + self.model_name + ".xml", self.tmp_dir + "model.xml" )
            shutil.copyfile( "./main.py", self.tmp_dir + "main.py" )


        # Move the tmp folder to results if not empty, else just remove the tmp file. 
        shutil.move( self.tmp_dir, C.SAVE_DIR  ) if len( os.listdir( self.tmp_dir ) ) != 0 else os.rmdir( self.tmp_dir )
        

    def set_camera_pos( self ):
        """
            Set the camera posture of the simulation. 
        """

        # Raw string to float. 
        tmp = str2float( self.args.cam_pos )

        # There should be six variables
        assert len( tmp ) == 6

        self.mj_viewer.cam.lookat[ 0 : 3 ] = tmp[ 0 : 3 ]
        self.mj_viewer.cam.distance        = tmp[ 3 ]
        self.mj_viewer.cam.elevation       = tmp[ 4 ]
        self.mj_viewer.cam.azimuth         = tmp[ 5 ]

    def set_ctrl( self, ctrl ):
        """
            For detailed controller description, please check 'controllers.py' 
        """
        self.ctrl = ctrl

    def set_obj( self, obj ):
        """ 
            Adding objective function refer to 'objectives.py for details' 
        """
        self.obj = obj
        
        # In case if the objective function is defined, set an array of objective value. 
        # The size of the array must be N = self.T / self.dt, the total number of time divided with the tiem step. 
        self.obj_arr = np.zeros( round( self.T / self.dt )  )

    def step( self ):
        """
            A wrapper function for our usage. 
        """

        # Update the step
        self.mj_sim.step( )

        # Update the number of steps and time
        self.n_steps += 1
        self.t = self.mj_data.time                               

    def run( self ):
        """ 
            Running the simulation 
        """

        if self.args.cam_pos is not None: self.set_camera_pos( ) 

        # The main loop of the simulation 
        while self.t <= self.T:

            # Render the simulation if mj_viewer exists        
            if self.mj_viewer is not None and self.n_steps % self.vid_step == 0:

                self.mj_viewer.render( )

                if self.is_record_vid: 
                    # Read the raw rgb image
                    rgb_img = self.mj_viewer.read_pixels( self.mj_viewer.width, self.mj_viewer.height, depth = False )

                    # Convert BGR to RGB and flip upside down.
                    rgb_img = np.flip( rgb_img, axis = 0 )
                    
                    # Add the frame list, this list will be converted to a video via moviepy library
                    self.frames.append( rgb_img )  

                # If reset button (BACKSPACE) is pressed
                if self.mj_viewer.is_reset:
                    self.reset( )
                    self.mj_viewer.is_reset = False
                
                # If SPACE BUTTON is pressed
                if self.mj_viewer.is_paused:    continue

            

            # [Calculate Input]
            # input_ref: The data array that are aimed to be inputted (e.g., qpos, qvel, qctrl etc.)
            # input_idx: The specific index of input_ref data array that should be inputted
            # input:     The actual input value which is inputted to input_ref
            if self.ctrl is not None: 
                input_ref, input_idx, input = self.ctrl.input_calc( self.t )
                input_ref[ input_idx ] = input

            # Run a single simulation 
            self.step( )

            # Set the objective function. This should be modified/ 
            if self.obj is not None: 
                self.obj_val = self.obj.output_calc( self.mj_model, self.mj_data, self.args )
                self.obj_arr[ self.n_steps - 1 ] = self.obj_val

            # Print the basic data
            if self.n_steps % self.print_step == 0:
                if not self.args.run_opt : print_vars( { "time": self.t,  "obj" : self.obj_val }  )

                if   self.args.save_data : 

                    # The data arrays that we will save
                    xpos_arr = [ self.mj_data.get_geom_xpos(  geom_name ) for geom_name in self.mj_model.geom_names ]
                    xvel_arr = [ self.mj_data.get_geom_xvelp( geom_name ) for geom_name in self.mj_model.geom_names ]
                    
                    print_vars( { "time": self.t, "qpos" : self.mj_data.qpos[ : ], "xpos" : xpos_arr, "xvel" : xvel_arr, "Jmat" : self.mj_data.get_site_jacp( "site_whip_COM" ).reshape( 3, -1 )[ :, 0 : 4  ], "obj" : self.obj_val }, save_dir = self.save_file  )

            # Check if simulation is stable. 
            # We check the accelerations
            if  self.is_sim_unstable( ):     
                print( '[UNSTABLE SIMULATION], HALTED AT {0:f} for a {1:f}-second long simulation'.format( self.t, self.T )  )                                 
                return "unstable"


    def is_sim_unstable( self ):
        """ 
            Check whether the simulation is stable. 
            If the acceleration exceeds some threshold value, then halting the simulation 
        """
        return True if max( abs( self.mj_data.qacc ) ) > 10 ** 6 else False

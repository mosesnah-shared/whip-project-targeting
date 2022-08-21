from xmlrpc.client import boolean
import numpy    as np
import scipy.io
from   modules.utils     import *
from   modules.MLmodule  import *
from   modules.constants import Constants as C

class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """

    def __init__( self, mj_sim, args, t_start: float ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data

        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args

        # Save the starting time of the simulation 
        self.t_start  = t_start 
        
        # A list of control parameters 
        self.ctrl_par_names = [ ] 

        # There are crucial parameters which can be calculated from the given model. 
        # Hence, "parsing" the xml model file
        self.parse_model( )


    def parse_model( self ):
        """
            Parsing and extracting out all the crucial parameters of the model
            e.g., The mass and inertia of the limbs.
        """

        # Just for the sake of abbreviation
        m = self.mj_model
        d = self.mj_data

        # ======== Extracting out the limb properties ========= #

        # Get the masses and inertias of the limbs. The name should contain "arm"
        # body_name2id change "name" to the index, and it is bassed to body_mass and body_inertia lists.
        # The name is all in "body_XXX_arm", hence we need to take out the "body" prefix
        limb_names = [ "_".join( name.split( "_" )[ 1 : ] ) for name in m.body_names if "body" and "arm" in name ]
        
        # get_model_par calls self.mj_model.body_mass attribute and get the value of body_name's 
        self.M  = { name: get_model_prop( m, "body", name, "mass"    ) for name in limb_names }
        self.I  = { name: get_model_prop( m, "body", name, "inertia" ) for name in limb_names }
        
        # Get the length of the limbs and the center of mass (COM)
        # Getting the length between the geoms. Note that order does not matter
        # Distance is measured from start site and end site, which is names as name_s, name_e
        # [1] Use np.linalg norm to calculate the distance 
        # [2] use site_pos[ site_name2id ] for the calculation. 
        # [3] L  is from 'site_XXX_start' to 'site_XXX_end' 
        # [4] Lc is from 'site_XXX_start' to 'site_XXX_COM' 
        #                                           from "site_limb_name_start"     to    "site_limb_name_end (COM)""
        self.L  = { name: get_length( m, d, "site", "_".join( [ name, "start" ]  ), "site", "_".join( [ name, "end" ] ) ) for name in limb_names } 
        self.Lc = { name: get_length( m, d, "site", "_".join( [ name, "start" ]  ), "site", "_".join( [ name, "COM" ] ) ) for name in limb_names }         

        # ====================================================== #

    def set_ctrl_par( self, **kwargs ):
        """
            Setting the control parameters

            Each controllers have their own controller parameters names (self.ctrl_par_names),

            This method function will become handy when we want to modify, or set the control parameters.

        """
        # Ignore if the key is not in the self.ctrl_par_names 
        # Setting the attribute of the controller 
        [ setattr( self, key, val ) for key, val in kwargs.items( ) if key in self.ctrl_par_names ]

    def input_calc( self, t ):
        """
            Calculating the torque input for the given time 
        """
        raise NotImplementedError

    def append_ctrl( self, ctrl ):
        """
            Append the current controller to ctrl1
            Usually we append the controller with convex combinations
        """
        raise NotImplementedError

    def get_tau_G( self ):
        """ 
            Calculate the gravity compensation torque for the model 
        """

        # Just for simplicity
        d, m = self.mj_data, self.mj_model

        # The gravity vector of the simulation
        self.g = m.opt.gravity           

        # Getting the number of actuators for the tau_G calculation 
        self.n_act = len( m.actuator_names )

        # Initialize the tau_G function 
        tau_G = np.zeros( self.n_act )

        # Get the mass of the whip, we simply add the mass with body name containing "whip"
        whip_node_names = [ "_".join( name.split( "_" )[ 1 : ] ) for name in m.body_names if "whip" in name ]
        self.M[ "whip" ] = sum( [ get_model_prop( m, "body", name, "mass" ) for name in whip_node_names ] )
        
        for name in [ "upper_arm", "fore_arm", "whip" ]:
            # Get the 3 x 4 Jacobian array, transpose it via .T method, and multiply the mass 
            tau_G += np.dot( d.get_site_jacp( "_".join( [ "site", name, "COM" ] ) ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ name ] * self.g  )
        
        return tau_G 

class JointImpedanceController( Controller ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_sim, mj_args, t_start: float = 0 ):

        super( ).__init__( mj_sim, mj_args, t_start )

        # Getting the number of actuators for the tau_G calculation 
        self.n_act = len( self.mj_model.actuator_names )

        # Define the controller parameters that we can change via "set_ctr_par" method
        self.ctrl_par_names = [ "K", "B", "q0i", "q0f", "D" ]       

        # Define the parameters that we will use for printing. This will be useful for "printing out the variables' detail" 
        self.print_par_naems = [ "K", "B", "q0" ]

        # Saving the data as an array
        self.is_save_data = mj_args.save_data

        if   self.n_act == 2:   
            c = 0.1
            self.K = C.K_2DOF
            self.B = c * self.K

        elif self.n_act == 4:  
            c = 0.05
            self.K = C.K_4DOF
            self.B = c * self.K

        # The trajectory 
        self.traj_pos, self.traj_vel = None, None

        # The movement parameters
        self.q0i, self.q0f, self.D, self.ti  = None, None, None, None

        # The number of control parameters of each name               
        # For K and B, we need self.n_act ** 2 parameters, in 2D
        # For pi and pf, we need self.n_act variables. 
        # For D, we need a single scalar
        self.n_ctrl_pars    = [ self.n_act ** 2, self.n_act ** 2, self.n_act, self.n_act, 1 ]    

        # If save data is True, define the array for saving the data 
        if self.is_save_data:

            # The number of samples required is simply as follows
            Ns = int( mj_args.run_time * mj_args.print_freq ) + 1

            # The time array 
            self.t_arr = np.zeros( Ns )

            # The Torque input array
            self.tau_arr = np.zeros( ( self.n_act, Ns ) )            
    
            # The current q (and q0) which is the joint (zero-torque) posture
            self.q_arr  = np.zeros( ( self.n_act, Ns ) )            
            self.q0_arr = np.zeros( ( self.n_act, Ns ) )                        

            # The current qdot (and q0dot) which is the joint (zero-torque) velocity 
            self.dq_arr  = np.zeros( ( self.n_act, Ns ) )                        
            self.dq0_arr = np.zeros( ( self.n_act, Ns ) )                        

            # The Jacobian matrix of the end-effector
            self.Jp_arr = np.zeros( ( 3, self.n_act, Ns ) )
            self.Jr_arr = np.zeros( ( 3, self.n_act, Ns ) )

            # The index for saving the data
            self.idx_data = 0                                    


    def set_traj( self, mov_pars: dict, basis_func: str = "min_jerk_traj" ):

        self.q0i = mov_pars[ "q0i" ]
        self.q0f = mov_pars[ "q0f" ]
        self.D   = mov_pars[  "D"  ]
        self.ti  = mov_pars[  "ti"  ]

        if   basis_func == "min_jerk_traj":
            self.traj_pos = lambda t :      self.q0i + ( self.q0f - self.q0i ) * (  10 * ( t / self.D ) ** 3 - 15 * ( t / self.D ) ** 4 +  6 * ( t / self.D ) ** 5 )
            self.traj_vel = lambda t :  1.0 / self.D * ( self.q0f - self.q0i ) * (  30 * ( t / self.D ) ** 2 - 60 * ( t / self.D ) ** 3 + 30 * ( t / self.D ) ** 4 )

        elif basis_func == "b_spline":
            pass

    def input_calc( self, t, is_gravity_comp = True, is_noise = False ):
        """
            Descriptions
            ------------
                We implement the controller. 
                The controller generates torque with the following equation 

                tau = K( q0 - q ) + B( dq0 - dq ) + tau_G 

                (d)q0: The zero-torque trajectory, which follows a minimum-jerk profile. 
                 (d)q: current angular position (velocity) of the robot

            Arguments
            ---------
                t: The current time of the simulation. 
        """

        # The following two trajectories  should not be "None"
        assert self.traj_pos and self.traj_vel

        # Get the current angular position and velocity of the robot 
        q  = self.mj_data.qpos[ 0 : self.n_act ]       
        dq = self.mj_data.qvel[ 0 : self.n_act ]
 
        if    t < self.ti:
            self.q0  = self.q0i 
            self.dq0 = np.zeros( self.n_act )

        elif  self.ti < t <= self.ti + self.D:
            self.q0  = self.traj_pos( t - self.t_start )
            self.dq0 = self.traj_vel( t - self.t_start )
        else:
            self.q0  = self.q0f
            self.dq0 = np.zeros( self.n_act )

        tau_imp = self.K @ ( self.q0 - q ) + self.B @ ( self.dq0 - dq )

        tau_G = self.get_tau_G( )                     if is_gravity_comp else np.zeros( self.n_act ) 
        tau_n = np.random.normal( size = self.n_act ) if is_noise        else np.zeros( self.n_act ) 

        self.tau  = tau_imp + tau_G + tau_n

        # Save the data if is_save_data Ture
        if self.is_save_data:
            
            if self.mj_sim.n_steps % self.mj_sim.print_step == 0:

                i = self.idx_data 

                # The current time and joint posture 
                self.t_arr[ i ] = t
                self.q_arr[ :, i ]  =  q
                self.dq_arr[ :, i ] = dq

                # The zero-torque trajectory
                self.q0_arr[ :, i ]  =  self.q0
                self.dq0_arr[ :, i ] = self.dq0           

                # The torque input 
                self.tau_arr[ :, i ] = self.tau 

                # The Jacobian matrix 
                self.Jp_arr[ :, :, i ] = self.mj_data.get_site_jacp( "site_fore_arm_end" ).reshape( 3, -1 )
                self.Jr_arr[ :, :, i ] = self.mj_data.get_site_jacr( "site_fore_arm_end" ).reshape( 3, -1 )

                # Update the data pointer 
                self.idx_data += 1


        # The  (1) object         (2) index array          (3) It's value. 
        return self.mj_data.ctrl, np.arange( self.n_act ), self.tau

    def save_data( self, dir_name ):
        """
            Save the controller details as a mat file
        """
        
        file_name = dir_name + "/ctrl.mat"
        
        # [TODO] There will be a single liner to do this, including this in the parent class
        scipy.io.savemat( file_name, { 'time': self.t_arr,    'qi': self.q0i,       'qf': self.q0f,
                                          'D': self.D,        'ti': self.ti,        'q': self.q_arr, 
                                         'dq': self.dq_arr,   'q0': self.q0_arr,  'dq0': self.dq0_arr, 
                                        'tau': self.tau_arr, 'Jp' : self.Jp_arr,  'Jr' : self.Jr_arr,
                                         'Kq': self.K,        'Bq': self.B }  )
                
    def reset( self ):
        """
            Initialize all variables  
        """
        self.traj_pos = None
        self.traj_vel = None 

        self.q0i, self.q0f, self.D = None, None, None

class CartesianImpedanceController( Controller ):

    def __init__( self, mj_model, mj_data, mj_args, t_start: float = 0 ):

        super( ).__init__( mj_model, mj_data, mj_args, t_start )

        # Getting the number of actuators 
        self.n_act = len( self.mj_model.actuator_names )

        # Define the controller parameters that we can change via "set_ctr_par" method
        self.ctrl_par_names = [ "Kx", "Bx", "x0i", "x0f", "D" ]       

        # Define the parameters that we will use for printing. This will be useful for "printing out the variables' detail" 
        self.print_par_naems = [ "K", "B", "q0" ]

    def set_traj( self, mov_pars: dict, basis_func: str = "min_jerk_traj" ):
        self.x0i = mov_pars[ "q0i" ]
        self.x0f = mov_pars[ "q0f" ]
        self.D   = mov_pars[  "D"  ]


        if   basis_func == "min_jerk_traj":
            self.traj_pos = lambda t :      self.x0i + ( self.x0f - self.x0i ) * (  10 * ( t / self.D ) ** 3 - 15 * ( t / self.D ) ** 4 +  6 * ( t / self.D ) ** 5 )
            self.traj_vel = lambda t :  1.0 / self.D * ( self.x0i - self.x0i ) * (  30 * ( t / self.D ) ** 2 - 60 * ( t / self.D ) ** 3 + 30 * ( t / self.D ) ** 4 )

        elif basis_func == "b_spline":
            pass

    def input_calc( self, t:float ):
        # The following two trajectories  should not be "None"
        assert self.traj_pos and self.traj_vel

        # Get the current angular position and velocity of the robot 
        q  = self.mj_data.qpos[ 0 : self.n_act ]       
        dq = self.mj_data.qvel[ 0 : self.n_act ]
 
        if    t < self.t_start:
            self.q0  = self.q0i 
            self.dq0 = np.zeros( self.n_act )

        elif  self.t_start < t <= self.t_start + self.D:
            self.q0  = self.traj_pos( t - self.t_start )
            self.dq0 = self.traj_vel( t - self.t_start )
        else:
            self.q0  = self.q0f
            self.dq0 = np.zeros( self.n_act )

        tau_imp = np.dot( self.K, self.q0 - q ) + np.dot( self.B, self.dq0 - dq )

        tau_G = self.get_tau_G( )                     
        tau_n = np.random.normal( size = self.n_act ) 

        self.tau  = tau_imp + tau_G + tau_n

        # The  (1) object         (2) index array          (3) It's value. 
        return self.mj_data.ctrl, np.arange( self.n_act ), self.tau

    def reset( self ):
        NotImplementedError( )

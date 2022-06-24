import numpy  as np

from   modules.constants import Constants as C
from   modules.utils     import length_elem2elem, get_property
from   itertools         import combinations

class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """

    def __init__( self, mj_model, mj_data, args, t_start: float ): 

        # Saving the reference of mujoco model and data for 
        self.mj_model = mj_model
        self.mj_data  = mj_data

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

        # ======== Extracting out the limb properties ========= #

        # Get the masses and inertias of the limbs. The name should contain "arm"
        # body_name2id change "name" to the index, and it is bassed to body_mass and body_inertia lists.
        # The name is all in "body_XXX_arm", hence we need to take out the "body" prefix
        limb_names = [ "_".join( name.split( "_" )[ 1 : ] ) for name in m.body_names if "body" and "arm" in name ]
        
        self.M  = { name: m.body_mass[    m.body_name2id[ name ] ] for name in limb_names }
        self.I  = { name: m.body_inertia[ m.body_name2id[ name ] ] for name in limb_names }
        
        # Get the length of the limbs and the center of mass (COM)
        # Getting the length between the geoms. Note that order does not matter
        # Distance is measured from start site and end site, which is names as name_s, name_e
        # [1] Use np.linalg norm to calculate the distance 
        # [2] use site_pos[ site_name2id ] for the calculation. 
        # [3] L  is from 'site_XXX_start' to 'site_XXX_end' 
        # [4] Lc is from 'site_XXX_start' to 'site_XXX_COM' 
        self.L  = { name: np.linalg.norm( m.site_pos[  m.site_name2id[ "_".join( [ 'site', name, 'start' ] )  ]  ], m.site_pos[  m.site_name2id[  "_".join( [ 'site', name, 'end' ] )  ]  ] , ord = 2  ) for name in limb_names } 
        self.Lc = { name: np.linalg.norm( m.site_pos[  m.site_name2id[ "_".join( [ 'site', name, 'start' ] )  ]  ], m.site_pos[  m.site_name2id[  "_".join( [ 'site', name, 'COM' ] )  ]  ] , ord = 2  ) for name in limb_names }         

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
        self.M[ "whip" ] = sum( [ m.body_mass( name ) for name in m.body_names if "whip" in name ] )

        for name in enumerate( [ "upper_arm", "fore_arm", "whip" ] ):
            # Get the 3 x 4 Jacobian array, transpose it via .T method, and multiply the mass 
            tau_G += np.dot( d.get_site_jacp( name ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ name ] * self.g  )
        
        return tau_G 

class JointImpedanceController( Controller ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_model, mj_data, mj_args, t_start: float = 0 ):

        super( ).__init__( mj_model, mj_data, mj_args, t_start )

        # Getting the number of actuators for the tau_G calculation 
        self.n_act = len( self.mj_model.actuator_names )

        # Define the controller parameters that we can change via "set_ctr_par" method
        self.ctrl_par_names = [ "K", "B", "pi", "pf", "D" ]       


        if   self.n_act == 2:   

            c = 0.1
            self.K = C.K_2DOF
            self.B = c * self.K

        elif self.n_act == 4:  
            c = 0.05
            self.K = C.K_4DOF
            self.B = c * self.K


        # The number of control parameters of each name               
        # For K and B, we need self.n_act ** 2 parameters, in 2D
        # For pi and pf, we need self.n_act variables. 
        # For D, we need a single scalar
        self.n_ctrl_pars    = [ self.n_act ** 2, self.n_act ** 2, self.n_act, self.n_act, 1 ]    


    def input_calc( self, t ):
        """
            Arguments
            ---------
                t: The current time of the simulation. 
        """
        if time <= 0:
            # For negative time, just set time as zero
            time = 0

        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]
        D  = self.traj.pars[ "D" ]

        if   time <= D:                                                         # If time greater than startTime
            self.x0, self.dx0 = self.traj.func_pos( time ), self.traj.func_vel( time  )  # Calculating the corresponding ZFT of the given time. the startTime should be subtracted for setting the initial time as zero for the ZFT Calculation.
        else:
            self.x0, self.dx0 = self.traj.pars[ "pf" ], np.zeros( ( self.n_act ) )       # Before start time, the posture should be remained at ZFT's initial posture

        tau_imp = np.dot( self.K, self.x0 - q ) + np.dot( self.B, self.dx0 - dq )
        tau_g   = self.get_G( )                                                 # Calculating the torque due to gravity compensation

        # If noise is on
        if self.is_noise:
            # Noise model is assumed to be white Gaussian Noise with variance proportional mean force / torque
            # [REF] Enoka, Roger M., and Dario Farina. "Force steadiness: from motor units to voluntary actions." Physiology 36.2 (2021): 114-130.
            tau   = tau_imp + tau_g

            #                           The mean as zeros,
            tau_n = np.random.normal( np.zeros( len( tau ) ), 1 * np.sqrt( np.abs( tau  ) )   )
            tau += tau_n
            self.tau = tau
            self.tau_n = tau_n
            # NotImplementedError( )
        else:
            self.tau   = tau_imp + tau_g

        # # [TMP] The result of lqr
        # # # print( tau_imp )
        # # + np.dot( 2, self.mjData.qpos[ -1 ] - np.pi ) + np.dot( 0.8, self.mjData.qvel[ -1 ] )# Calculating the torque due to impedance
        # # tau_imp = np.dot( 1, self.mjData.qpos[ -1 ] - np.pi ) + np.dot( 0.4, self.mjData.qvel[ -1 ] )
        return self.mjData.ctrl, self.idx_act, self.tau

if __name__ == "__main__":
    pass

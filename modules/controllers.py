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
    def __init__( self, mj_sim, args ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data

        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args        

        # Get the number of actuators of the model
        self.n_act = len( self.mj_model.actuator_names )        

        # Get the number of generalized coordinate of the model
        self.nq = len( self.mj_model.joint_names )                

        # The list of controller parameters
        self.names_ctrl_pars = None

        # Get the number of geom names of the simulation
        self.n_geoms = len( self.mj_model.geom_names )        

        # Saving the name of the data as a string array for "save_data" method
        self.names_data = None

        # Saving the size of the data that we will save for an array
        # The input is either a scalar or a 2D, 3D tuple (e.g., (3,2) )
        self.N_data = None        


    def init_ctrl_pars( self ):
        """
            Initialize an empty list of conroller parameters
        """

        assert self.names_ctrl_pars is not None

        [ setattr( self, n, [ ] ) for n in self.names_ctrl_pars  ]


    def init_save_data( self ):
        """
            Initialize an multi-dimensional array for saving the data. 
        """
        # The number of samples required is simply as follows
        Ns = int( self.mj_args.run_time * self.mj_args.print_freq ) + 1

        # The number of names of data should match the length of N_data
        assert len( self.names_data ) == len( self.N_data )

        # The size of the array 
        # Simply concatenate the tuple or scalar at N_data with Ns
        N_data_size = [ ( Ns , ) + ( x if isinstance( x, tuple ) else ( x , ) ) for x in self.N_data ] 
        [ setattr( self, name + "_arr", np.zeros( size ) ) for name, size in zip( self.names_data, N_data_size ) ]

        # Set the pointer of the saved data
        self.idx_data = 0

    def save_data( self ):
        """
            Update the saved data
        """

        assert self.mj_args.is_save_data

        for name in self.names_data:
            tmp_attr = getattr( self, name + "_arr" )
            tmp_attr[ self.idx_data, : ] = getattr( self, name )
   
        self.idx_data += 1

    def export_data( self, dir_name ):
        """
            Export the data as mat file
        """
        file_name = dir_name + "/ctrl.mat"
        
        # Packing up the arrays 
        dict1 = { name + "_arr": getattr( self, name + "_arr" ) for name in self.names_data }

        # Packing up the controller parameters 
        dict2 = { name: getattr( self, name )          for name in self.names_ctrl_pars }
        
        # Merging the two dictionries 
        dict1.update( dict2 )

        scipy.io.savemat( file_name, dict1 )

    def input_calc( self, t ):
        """
            Calculating the torque input for the given time 
        """
        raise NotImplementedError


class ImpedanceController( Controller ):
    """
        Description:
        -----------
            Parent class for the Impedance Controller

    """

    def __init__( self, mj_sim, args ): 

        super( ).__init__( mj_sim, args )

        # There are crucial parameters which can be calculated from the given model. 
        # Hence, "parsing" the xml model file
        # The model name should be within the following list 
        assert args.model_name in [ "2D_model", "2D_model_w_whip", "3D_model", "3D_model_w_whip" ]

        # Parsing the model to get the mass, inertia, length and the COM position.
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

    def get_tau_G( self ):
        """ 
            Calculate the gravity compensation torque for the model 
        """
        
        # Just for simplicity
        d, m = self.mj_data, self.mj_model

        # The gravity vector of the simulation
        self.g = m.opt.gravity           

        # Initialize the tau_G function 
        tau_G = np.zeros( self.n_act )

        # Get the mass of the whip, we simply add the mass with body name containing "whip"
        whip_node_names = [ "_".join( name.split( "_" )[ 1 : ] ) for name in m.body_names if "whip" in name ]
        self.M[ "whip" ] = sum( [ get_model_prop( m, "body", name, "mass" ) for name in whip_node_names ] )
        
        for name in [ "upper_arm", "fore_arm", "whip" ]:
            # Get the 3 x 4 Jacobian array, transpose it via .T method, and multiply the mass 
            tau_G += np.dot( d.get_site_jacp( "_".join( [ "site", name, "COM" ] ) ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ name ] * self.g  )
        
        return tau_G 

class JointImpedanceController( ImpedanceController ):

    """
        Description:
        ----------
            Class for a Joint (i.e., Generalized coordinate) Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mj_sim, mj_args ):

        super( ).__init__( mj_sim, mj_args )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kq", "Bq", "q0i", "q0f", "D", "ti" )

        # Generate an empty list with the corresponding controller parameters names
        self.init_ctrl_pars( )

        # The number of submovements
        self.n_movs = 0 

        # The name of variables that will be saved 
        self.names_data = ( "t",      "tau",         "q",       "q0",     "dq",        "dq0",             "Jp",            "Jr"  )
        self.N_data     = (   1, self.n_act,    self.nq, self.n_act,  self.nq,   self.n_act,  (3, self.n_act), (3, self.n_act)   )

        # If save data is True, define the array for saving the data 
        if self.mj_args.is_save_data: self.init_save_data( )

    def set_impedance( self, Kq: np.ndarray, Bq:np.ndarray ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( Kq      ) == self.n_act and len( Bq      ) == self.n_act 
        assert len( Kq[ 0 ] ) == self.n_act and len( Bq[ 0 ] ) == self.n_act 

        self.Kq = Kq
        self.Bq = Bq 

    def set_mov_pars( self, q0i : np.ndarray, q0f: np.ndarray, D:float, ti: float ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( q0i ) == self.n_act and len( q0f ) == self.n_act
        assert D > 0 and ti >= 0

        # If done, append the mov_parameters
        self.q0i.append( q0i )
        self.q0f.append( q0f )
        self.D.append(     D )
        self.ti.append(   ti )

        self.n_movs += 1

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
        assert self.Kq is not None and self.Bq is not None
        assert self.n_movs >= 1 

        # Save the current time 
        self.t = t 

        # Get the current angular position and velocity of the robot arm only
        self.q  = self.mj_data.qpos[ : self.n_act ]
        self.dq = self.mj_data.qvel[ : self.n_act ]
 
        self.q0  = np.zeros( self.n_act )
        self.dq0 = np.zeros( self.n_act )

        for i in range( self.n_movs ):
            for j in range( self.n_act ):
                tmp_q0, tmp_dq0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                self.q0[ j ]  += tmp_q0 
                self.dq0[ j ] += tmp_dq0

        tau_imp = self.Kq @ ( self.q0 - self.q ) + self.Bq @ ( self.dq0 - self.dq )

        self.Jp = self.mj_data.get_site_jacp( "site_whip_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ]
        self.Jr = self.mj_data.get_site_jacr( "site_whip_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ]

        tau_G = self.get_tau_G( )                     if is_gravity_comp else np.zeros( self.n_act ) 
        tau_n = np.random.normal( size = self.n_act ) if is_noise        else np.zeros( self.n_act ) 

        self.tau  = tau_imp + tau_G + tau_n

        # Save the data if is_save_data Ture
        if self.mj_args.is_save_data and self.mj_sim.n_steps % self.mj_sim.save_step == 0:  
            self.save_data( )

        # The  (1) object         (2) index array          (3) It's value. 
        return self.mj_data.ctrl, np.arange( self.n_act ), self.tau

                
    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init_ctrl_pars( )
        self.init_save_data( )

        self.n_mov = 0 

class CartesianImpedanceController( ImpedanceController ):

    def __init__( self, mj_sim, mj_args ):
        raise NotImplementedError( )

    def input_calc( self, t:float ):
        raise NotImplementedError( )

    def reset( self ):
        raise NotImplementedError( )


class SphereController:

    def __init__( self, mj_sim, args ): 
        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim.mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data
        # Saving the arguments passed via ArgumentParsers
        self.mj_args  = args

    def set_initial_orientation( self, q_init: np.ndarray ):
        """
            Set random initial orientation.
        """
        assert len( q_init ) == 3
        self.mj_data.qpos[ : ] = q_init[ : ]
        self.mj_sim.mj_sim.forward( )

    def set_desired_orientation( self, o_des ):
        """
            The desired orientation in R matrix form 
        """
        # Assert 
        self.o_des = o_des

    def input_calc( self ):
        """
            Setting the desired orientation of the robot 
        """
        # Get the current angular position and velocity of the robot 
        q  = self.mj_data.qpos[ : ]       
        dq = self.mj_data.qvel[ : ]
        # 
 
        # The  (1) object         (2) index array          (3) It's value. 
        return self.mj_data.ctrl, np.arange( self.n_act ), np.zeros( 3 )

    def reset( self ):
        """
            Initialize all variables  
        """
        NotImplementedError( )

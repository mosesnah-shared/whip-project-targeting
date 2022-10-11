import numpy    as np
import scipy.io
from   modules.utils     import quat2rot, quat2angx, rot2quat, get_model_prop, get_length, min_jerk_traj, skew_sym

class Controller:
    """
        Description:
        -----------
            Parent class for the controllers

    """
    def __init__( self, mj_sim, args, name ): 

        # Saving the reference of mujoco model and data for 
        self.mj_sim   = mj_sim
        self.mj_model = mj_sim.mj_model
        self.mj_data  = mj_sim.mj_data
        self.name     = name

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

    def init( self ):
        """
            Initialize an empty list of conroller parameters
        """

        assert self.names_data      is not None
        assert self.names_ctrl_pars is not None 
        
        # Generate an empty arrays with data and controller parameters 
        [ setattr( self, name + "_arr", [ ] ) for name in self.names_data       ]
        [ setattr( self,          name, [ ] ) for name in self.names_ctrl_pars  ]

    def save_data( self ):
        """
            Update the saved, which will be defined in method "input_calc"
        """
        
        for name in self.names_data:
            val = getattr( self, name )
            getattr( self, name + "_arr" ).append( val )
        

    def export_data( self, dir_name ):
        """
            Export the data as mat file
        """
        file_name = dir_name + "/ctrl_" + self.name + ".mat"
        
        # Packing up the arrays as a dictionary
        dict1 = { name + "_arr": getattr( self, name + "_arr" ) for name in self.names_data      }
        dict2 = { name: getattr( self, name )                   for name in self.names_ctrl_pars }
        
        scipy.io.savemat( file_name, { **dict1, **dict2 } )

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

    def __init__( self, mj_sim, args, name ): 

        super( ).__init__( mj_sim, args, name )

        # There are crucial parameters which can be calculated from the given model. 
        # Hence, "parsing" the xml model file
        # The model name should be within the following list 

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

    def __init__( self, mj_sim, mj_args, name ):

        super( ).__init__( mj_sim, mj_args, name )

        # The name of the controller parameters 
        self.names_ctrl_pars = ( "Kq", "Bq", "q0i", "q0f", "D", "ti" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "tau", "q", "q0", "dq", "dq0", "Jp", "Jr"  )

        # Generate an empty lists names of parameters
        self.init( )

        # The number of submovements
        self.n_movs = 0 

    def set_impedance( self, Kq: np.ndarray, Bq:np.ndarray ):

        # Make sure the given input is (self.n_act x self.n_act )
        assert len( Kq      ) == self.n_act and len( Bq      ) == self.n_act 
        assert len( Kq[ 0 ] ) == self.n_act and len( Bq[ 0 ] ) == self.n_act 

        self.Kq = Kq
        self.Bq = Bq 

    def add_mov_pars( self, q0i : np.ndarray, q0f: np.ndarray, D:float, ti: float ):

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
        self.q  = np.copy( self.mj_data.qpos[ : self.n_act ] )
        self.dq = np.copy( self.mj_data.qvel[ : self.n_act ] )
 
        self.q0  = np.zeros( self.n_act )
        self.dq0 = np.zeros( self.n_act )

        for i in range( self.n_movs ):
            for j in range( self.n_act ):
                tmp_q0, tmp_dq0 = min_jerk_traj( t, self.ti[ i ], self.ti[ i ] + self.D[ i ], self.q0i[ i ][ j ], self.q0f[ i ][ j ], self.D[ i ] )

                self.q0[ j ]  += tmp_q0 
                self.dq0[ j ] += tmp_dq0

        tau_imp = self.Kq @ ( self.q0 - self.q ) + self.Bq @ ( self.dq0 - self.dq )

        self.Jp = np.copy( self.mj_data.get_site_jacp( "site_whip_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ] )
        self.Jr = np.copy( self.mj_data.get_site_jacr( "site_whip_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ] )

        tau_G = self.get_tau_G( )                     if is_gravity_comp else np.zeros( self.n_act ) 
        tau_n = np.random.normal( size = self.n_act ) if is_noise        else np.zeros( self.n_act ) 

        self.tau  = tau_imp + tau_G + tau_n

        #     (1) index array       (3) The tau value
        return  np.arange( self.n_act ), self.tau

                
    def reset( self ):
        """
            Initialize all ctrl variables  
        """

        self.init( )
        self.n_movs = 0 

class CartesianImpedanceController( ImpedanceController ):

    def __init__( self, mj_sim, mj_args ):
        raise NotImplementedError( )

    def input_calc( self, t:float ):
        raise NotImplementedError( )

    def reset( self ):
        raise NotImplementedError( )


class SphereController( Controller ):

    def __init__( self, mj_sim, args, name ): 
                
        super( ).__init__( mj_sim, args, name )

        self.names_ctrl_pars = ( "k", "b", "R_des" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "q", "dq", "tau", "R", "theta", "Jr", "quat_cur", "R_cur"  )

        # Generate an empty lists names of parameters
        self.init( )        

    def set_desired_orientation( self, R_des ):
        """
            The desired orientation in R matrix form 
        """
        # Assert that o_des is a rotation matrix

        self.R_des = R_des

    def input_calc( self, t ):
        """
            Setting the desired orientation of the robot 
        """

        self.t = t

        self.k = 3.0
        self.b = 0.2

        # Get the current angular position and velocity of the robot 
        self.q  = np.copy( self.mj_data.qpos[ : ] )
        self.dq = np.copy( self.mj_data.qvel[ : ] )

        self.Jr = np.copy( self.mj_data.get_body_jacr( "sphere" ).reshape( 3, -1 ) )

        # The w is simply Jr dq
        w = self.Jr @ self.dq

        # Get the Rotation matrix difference 
        self.quat_cur = np.copy( self.mj_data.get_body_xquat( "sphere" ) )
        self.R_cur  = quat2rot( self.quat_cur )
        self.R = self.R_cur
        R_diff = self.R_cur.T @ self.R_des

        # Get the axis of rotation
        theta, axis = quat2angx( rot2quat( R_diff ) )

        self.theta = theta

        axis_world = self.R_cur @ axis 
        m = axis_world * self.k * theta - self.b * w
        self.tau = self.Jr.T @ m


        # The  (1) index array          (2) It's value. 
        return np.arange( self.n_act ), self.tau

    def reset( self ):
        NotImplementedError( )



class SphereControllerAdvanced( Controller ):

    def __init__( self, mj_sim, args, name ): 
                
        super( ).__init__( mj_sim, args, name )

        self.names_ctrl_pars = ( "K", "B" , "R_des" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "q", "dq", "tau", "R_cur" )

        # Generate an empty lists names of parameters
        self.init( )        

    def set_desired_orientation( self, R_des ):
        """
            The desired orientation in R matrix form 
        """
        # Assert that o_des is a rotation matrix

        self.R_des = R_des

    def input_calc( self, t ):  
        """
            Setting the desired orientation of the robot 
        """

        self.t = t

        self.K = 10 * np.eye( 3 )
        self.B = 0.1 * self.K

        # Get the current angular position and velocity of the robot 
        self.q  = np.copy( self.mj_data.qpos[ : ] )
        self.dq = np.copy( self.mj_data.qvel[ : ] )

        self.Jr = self.mj_data.get_body_jacr( "sphere" ).reshape( 3, -1 )

        # The w is simply Jc dq
        w = self.Jr @ self.dq

        # Get the Rotation matrix difference 
        self.R_cur  = np.copy( quat2rot( self.mj_data.get_body_xquat( "sphere" ) ) )
        R_diff = self.R_des.T @ self.R_cur 

        Q_diff = rot2quat( R_diff )

        # With respect to the current frame
        eta = Q_diff[ 0 ]
        eps_r = Q_diff[ 1: ]

        E = eta * np.eye( 3 ) - skew_sym( eps_r )

        Kprime = 2 * E.T @ self.K
        m_cur = Kprime @ eps_r

        m = - self.R_cur @ m_cur - self.B @ w

        self.tau = self.Jr.T @ m

        # The  (1) index array          (2) It's value. 
        return np.arange( self.n_act ), self.tau

    def reset( self ):
        NotImplementedError( )



class Excitator( Controller ):
    """
        Description:
        ----------
            Controller class for an Exciting a Whip

    """
    def __init__( self, mj_sim, args, name:str ):
        super( ).__init__( mj_sim, args, name )

        self.names_ctrl_pars = ( "A", "w", "ti" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "x", "dx", "q", "dq", "pos" )

        # Generate an empty lists names of parameters
        self.init( )        

    def set_mov_pars( self, A:float, w:float, ti:float ):

        assert A > 0 and w > 0 and ti >= 0

        self.A = A
        self.w = w
        self.ti = ti 

    def input_calc( self, t ):

        self.t = t

        if t <= self.ti:
            pos = 0

        elif t >= self.ti and t <= self.ti + 2 * self.w:
            pos = self.A * ( 1 - np.cos( np.pi / self.w * ( t - self.ti ) ) )
            
        else:
            pos = 0

        self.x  = np.copy( [ self.mj_data.get_geom_xpos(  name ) for name in self.mj_model.geom_names ] )
        self.dx = np.copy( [ self.mj_data.get_geom_xvelp( name ) for name in self.mj_model.geom_names ] )
        
        self.q  = np.copy( self.mj_data.qpos[ : ] )
        self.dq = np.copy( self.mj_data.qvel[ : ] )

        self.pos = pos
        

        return np.array( [ 0 ] ), pos



class Excitator_MJT( Controller ):
    """
        Description:
        ----------
            Controller class for an Exciting a Whip

    """
    def __init__( self, mj_sim, args, name:str ):
        super( ).__init__( mj_sim, args, name )

        self.names_ctrl_pars = ( "dpos", "D", "ti" )

        # The name of variables that will be saved 
        self.names_data = ( "t", "x", "dx", "q", "dq", "pos" )

        # Generate an empty lists names of parameters
        self.init( )        

    def set_mov_pars( self, dpos: float, D:float, ti:float ):

        assert dpos >= 0 and D >= 0 and ti >= 0 

        self.dpos = dpos
        self.D    = D
        self.ti   = ti

    def input_calc( self, t ):

        self.t  = t

        self.x  = np.copy( [ self.mj_data.get_geom_xpos(  name ) for name in self.mj_model.geom_names ] )
        self.dx = np.copy( [ self.mj_data.get_geom_xvelp( name ) for name in self.mj_model.geom_names ] )

        self.q  = np.copy( self.mj_data.qpos[ : ] )
        self.dq  = np.copy( self.mj_data.qvel[ : ] )

        self.pos, _ = min_jerk_traj( t, self.ti, self.ti + self.D, 0, self.dpos, self.D )       

        return np.array( [ 0 ] ), self.pos

if __name__ == "__main__":
    pass

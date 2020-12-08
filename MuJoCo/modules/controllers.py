# [Built-in modules]

# [3rd party modules]
import numpy as np
import time
import pickle

from modules.utils        import my_print
import matplotlib.pyplot as plt

try:
    import mujoco_py as mjPy

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

# Added
try:
    import sympy as sp
    from sympy.utilities.lambdify import lambdify, implemented_function

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install sympy, \
                                             Simply type pip3 install sympy. \
                                             Sympy is necessary for building ZFT Calculation)".format( e ) )

# [Local modules]


class Controller( ):
    """
        Description:
        -----------
            Parent class for the controllers
    """


    def __init__( self, mjModel, mjData ):
        """

        """
        self.mjModel        = mjModel
        self.mjData         = mjData
        self.ctrl_par_names = None


    def set_ctrl_par( self, **kwargs ):
        """
            Setting the control parameters

            Each controllers have their own controller parameters names (self.ctrl_par_names),

            This method function will become handy when we want to modify, or set the control parameters.

        """
        if kwargs is not None:
            for args in kwargs:
                if args in self.ctrl_par_names:
                    setattr( self, args, kwargs[ args ] )
                else:
                    pass

    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.


class ImpedanceController( Controller ):
    """
        Description:
        ----------
            Class for an Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mjModel, mjData, is_grav_comps = True ):

        super().__init__( mjModel, mjData )

        self.act_names      = mjModel.actuator_names                            # The names of the actuators, all the names end with "TorqueMotor" (Refer to xml model files)
        self.n_act          = len( mjModel.actuator_names )                     # The number of actuators, 2 for 2D model and 4 for 3D model
        self.idx_act        = np.arange( 0, self.n_act )                        # The idx array of the actuators, this is useful for self.input_calc method
        self.n_limbs        = '-'.join( mjModel.body_names ).lower().count( 'arm' ) # The number of limbs of the controller. Checking bodies which contain "arm" as the name (Refer to xml model files)
        self.is_grav_comps  = is_grav_comps                                     # Boolean for gravity compensation. [True-1-ON] [False-0-OFF]
        self.g              = mjModel.opt.gravity                               # The gravity vector of the simulation

        # Controller uses first-order impedance controller. Hence the position/velocity of the ZFT(Zero-torque trajectory) must be defined
        self.ZFT_func_pos   = None
        self.ZFT_func_vel   = None


        # Mass information of the limbs
        if self.n_limbs == 2:                                                   # For arm model with 2 limbs.
            bodyName  = ['upperArm', 'foreArm' ]                                # Masses of the body that are needed for the gravity compensation
            self.mUA, self.mFA  = [ self.mjModel.body_mass[ idx ]               # The mass for each "bodyName" are saved in MVec, the name of the bodies are defined in the "xml" model file.
                                    for idx, s in enumerate( self.mjModel.body_names ) if s in bodyName ]   # mUA: Mass of the upperArm Limb
                                                                                                            # mUA: Mass of the foreArm Limb

            self.Mw = sum( self.mjModel.body_mass[ : ] ) - self.mUA - self.mFA  #  Mw: Mass of the total whip model. This is used for gravity compensation.
                                                                                #  Mw is zero when there is no whip

        elif self.n_limbs == 3:
            raise NotImplementedError( )

        # The actuators
        if not all( 'torque' in tmp.lower() for tmp in mjModel.actuator_names ):
            raise ValueError( "For Impedance Controller, the model actuator doesn't contain 'torque' actuators. \
                               Please check whether the model and controller object corresponds with each other correctly")


        # [IMPEDANCE PARAMETERS FOR THE 2DOF/4DOF UPPER-LIMB MODEL]
        # The Stiffness matrix K and Damping matrix B will be used for the following impedance torque controller:
        # torque = K ( phi - theta ) + B ( dphi - dtheta)
        # [REF #1] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 2.3]   Mechanical Impedances.
        # [REF #2] [Moses C. Nah] [MIT Master's Thesis]: "Ibid",                                              [Section 7.2.1] Impedance Controller
        # [REF #3] [Moses C. Nah] [MIT Master's Thesis]: "Ibid",                                              [Section 8.2.1] Impedance Controller

        if   self.n_act == 2:   # 2DOF Robot

            c = 0.1
            self.K = np.array( [ [ 29.50, 14.30 ] ,
                                 [ 14.30, 39.30 ] ] )
            self.B = c * self.K

        elif self.n_act == 4:   # 4DOF Robot

            c = 0.05
            self.K = np.array( [ [ 17.40, 4.70, -1.90, 8.40 ] ,
                                 [  9.00, 33.0,  4.40, 0.00 ] ,
                                 [ -13.6, 3.00,  27.7, 0.00 ] ,
                                 [  8.40, 0.00,  0.00, 23.2 ] ] )
            self.B = c * self.K


        else:
            raise ValueError( "Number of Actuators seem wrong, Please Check .xml Model File.")

        # [2DOF Robot] 5 movement parameters in total - Intial posture (2), Final posture (2) and duration (1)
        # [4DOF RObot] 9 movement parameters in total - Intial posture (4), Final posture (4) and duration (1)
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.3.] Implementation
        # [REF] [Moses C. Nah] Ibid.        ,                                                              [Section 8.2.2.] Zero-Torque Trajectory
        self.n_mov_pars     = 2 * self.n_act + 1                                    # 2 *, for initial/final postures and +1 for movement duration
        self.mov_parameters = None                                                  # The actual values of the movement parameters, initializing it with random values
        self.n_ctrl_pars    = [ self.n_mov_pars, self.n_act ** 2, self.n_act ** 2 ] # The number of ctrl parameters. This definition would be useful for the optimization process.
                                                                                    # K and B has n^2 elements, hence self.n_act ** 2


        self.ctrl_par_names = [ "mov_parameters", "K", "B" ]                    # Useful for self.set_ctrl_par method
        self.t_sym = sp.symbols( 't' )                                          # time symbol of the equation


    def gravity_compensation( self ):

        if self.is_grav_comps:

            if   self.n_limbs == 2:

                # Torque for Gravity compensation is simply tau = J^TF
                # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
                tau_g = np.dot( self.mjData.get_site_jacp( "upperArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].transpose(), - self.mUA * self.g  )  \
                      + np.dot( self.mjData.get_site_jacp(  "foreArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].transpose(), - self.mFA * self.g  )  \
                      + np.dot( self.mjData.get_geom_jacp(  "geom_EE"    ).reshape( 3, -1 )[ :, 0 : self.n_act ].transpose(), - self.Mw  * self.g  )

            elif self.n_limbs == 3:
                raise NotImplementedError( )
        else:
            tau_g = np.zeros( self.n_act )

        return tau_g


    def set_ZFT( self ):
        """
            Description:
            ----------
                Setting the ZFT(Zero-torque trajectory, strictly speaking it should be ZTT, but ZFT is much popular usage :)
                This method is only called once "before" running the simulation, and "after" the self.mov_parameters are well-defined

        """


        # Defining the equations for the ZFT, this function must be done before the actual simulation
        if self.mov_parameters is None:
            raise ValueError( "Movement parameters are not defined")

        pi = np.array( self.mov_parameters[          0 : self.n_act     ] )     # Initial Posture
        pf = np.array( self.mov_parameters[ self.n_act : 2 * self.n_act ] )     # Final   Posture
        D  = self.mov_parameters[ -1 ]                                          # Duration it took from start to end

        # Basis function used for the ZFT Trajectory is minimum-jerk-trajectory
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.2.] Zero-torque trajectory
        # [REF] [T. Flash and N. Hogan]             : "Flash, Tamar, and Neville Hogan. "The coordination of arm movements: an experimentally confirmed mathematical model."
        self.ZFT_func_pos = pi + ( pf - pi ) * ( 10 * np.power( self.t_sym ,3 ) / ( D ** 3 ) - 15 * np.power( self.t_sym , 4 ) / ( D ** 4 ) +  6 * np.power( self.t_sym , 5 ) / ( D ** 5 ) )
        self.ZFT_func_vel = [ sp.diff( tmp, self.t_sym ) for tmp in self.ZFT_func_pos ]

        # Lambdify the functions
        # [TIP] This is necessary for computation Speed!
        self.ZFT_func_pos = lambdify( self.t_sym, self.ZFT_func_pos )
        self.ZFT_func_vel = lambdify( self.t_sym, self.ZFT_func_vel )


    def get_ZFT( self, time ):

        D = self.mov_parameters[ -1 ]                                           # Last element is duration
        t = D if time >= D else time                                            # Rectifying the time value if time is larger than D
                                                                                # This means that the ZFT of the controller remains at final posture.
        phi  = np.array( self.ZFT_func_pos( t ) )
        dphi = np.array( self.ZFT_func_vel( t ) )


        return phi, dphi

    def input_calc( self, start_time, current_time ):
        """

        """

        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]

        if   current_time >= start_time:                                        # If time greater than startTime
            self.phi, self.dphi = self.get_ZFT( current_time - start_time  )    # Calculating the corresponding ZFT of the given time. the startTime should be subtracted for setting the initial time as zero for the ZFT Calculation.
        else:
            self.phi, self.dphi = q, dq                                         # Before start time, the posture should be remained at ZFT's initial posture

        tau_imp = np.dot( self.K, self.phi - q ) + np.dot( self.B, self.dphi - dq ) # Calculating the torque due to impedance
        tau_g   = self.gravity_compensation( )                                      # Calculating the torque due to gravity compensation

        return self.mjData.ctrl, self.idx_act, tau_imp  + tau_g


if __name__ == "__main__":
    pass

# [Built-in modules]

# [3rd party modules]
import numpy as np
import sys
import time
import pickle

from   modules.utils        import my_print, get_elem_type, length_elem2elem, get_property
from   modules.traj_funcs   import MinJerkTrajectory
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

            For all of the controllers, there exist some sort of nominal trajectory (e.g., zero-force traj, zero-torque traj etc.).
            Hence, a lambdified function should exist, which can calculate the position of nominal trajectory at arbitrary time.

            Moreover, since the controller must identify some model parameters of our upper-limb model, there should be a "parse" function
            that parses the "mjModel" class and retrieves the essential data (e.g., number of actuators, number of limbs etc. )

            For the parsing, some member functions which faciliate this are required, and hence defined in this controller class.

            From this parent class, the children classes are subsequently defined, and the input_calc function are augmented.

    """


    def __init__( self, mjModel, mjData, mjArgs ):
        """

        """
        self.mjModel        = mjModel
        self.mjData         = mjData
        self.mjArgs         = mjArgs
        self.ctrl_par_names = None

        # Parsing the current model that we are using.
        self.parse_model( )

        # The symbolic lambdified function of the trajectory to track.
        # Trajectory is defined under "modules.traj_funcs"
        self.traj = None

    def parse_model( self ):
        # [Basic Parameters of the model]
        # The number of actuators, number of limbs should be calculated.
        # Mostly the upper-limb model parameters
        self.act_names      = self.mjModel.actuator_names                       # The names of the actuators, all the names end with "TorqueMotor" (Refer to xml model files)
        self.n_act          = len( self.mjModel.actuator_names )                # The number of actuators, 2 for 2D model and 4 for 3D model
        self.n_limbs        = '-'.join( self.mjModel.body_names ).lower().count( 'arm' ) # The number of limbs of the controller. Checking bodies which contain "arm" as the name (Refer to xml model files)

        self.idx_act        = np.arange( 0, self.n_act )                        # The idx array of the actuators, this is useful for self.input_calc method
        self.g              = self.mjModel.opt.gravity                          # The gravity vector of the simulation

        if   self.n_limbs == 2:
            self.M  = [ get_property( self.mjModel, 'body_upper_arm', 'mass'    ), get_property( self.mjModel, 'body_fore_arm', 'mass'    ) ]
            self.I  = [ get_property( self.mjModel, 'body_upper_arm', 'inertia' ), get_property( self.mjModel, 'body_fore_arm', 'inertia' ) ]
            self.L  = [ length_elem2elem( self.mjModel, self.mjData, 'geom_shoulder', 'geom_elbow'        ), length_elem2elem( self.mjModel, self.mjData, 'geom_elbow' , 'geom_end_effector'  )  ]
            self.Lc = [ length_elem2elem( self.mjModel, self.mjData, 'geom_shoulder', 'site_fore_arm_COM' ), length_elem2elem( self.mjModel, self.mjData, 'geom_elbow' , 'site_upper_arm_COM' )  ]

        elif self.n_limbs == 3:
            NotImplementedError( )

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

    def input_calc( self, time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError( )                                            # Adding this NotImplementedError will force the child class to override parent's methods.

    def append_ctrl( self, ctrl1 ):
        """
            Append the current controller to ctrl1
            Usually we append the controller with convex combinations
        """
        raise NotImplementedError( )                                            # Adding this NotImplementedError will force the child class to override parent's methods.



    def get_G( self ):

        if   self.n_limbs == 2:

            # Torque for Gravity compensation is simply tau = J^TF
            # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
            G = np.dot( self.mjData.get_site_jacp( "site_upper_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[0] * self.g  )  \
              + np.dot( self.mjData.get_site_jacp(  "site_fore_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[1] * self.g  )

            # The mass of the whip is the other masses summed up
            self.Mw = sum( self.mjModel.body_mass[ : ] ) - sum( self.M[ : ] )

            # If no whip is attached, then the mass will be zero.
            G += np.dot( self.mjData.get_geom_jacp(  "geom_end_effector"    ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.Mw  * self.g  )

        elif self.n_limbs == 3:
            raise NotImplementedError( )


        return G

class DebugController( Controller ):
    """
        Description:
        ----------
            Controller for quick debugging, useful when practicing/debugging with MuJoCo

    """
    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )
        self.n_act = 0

    def set_ZFT( self ):
        return 0

    def input_calc( self, current_time ):
        return None, None, 0



class ImpedanceController( Controller ):
    """
        Description:
        ----------
            Class for an Impedance Controller
            Inheritance of parent class "Contronller"

    """
    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )

        # The impedance parameter of the controller
        # Used for both Cartesian or Joint impedances.
        self.Kmat = None
        self.Bmat = None
        self.Mmat = None

        self.n_mov_pars     = None                                              # The number of parameters of the movement
        self.n_ctrl_pars    = None                                              # The number of ctrl parameters. This definition would be useful for the optimization process.
        self.mov_parameters = None                                              # The actual values of the movement parameters, initializing it with random values
        self.ctrl_par_names = None                                              # Useful for self.set_ctrl_par method

class JointImpedanceController( ImpedanceController ):

    """
        Description:
        ----------
            Class for a Joint Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )


        if   self.n_act == 2:   # The default K and B matrices used for the 2-DOF Robot
                                # [REF] Nah, Moses C., et al. "Dynamic primitives facilitate manipulating a whip."
                                # 2020 8th IEEE RAS/EMBS International Conference for Biomedical Robotics and Biomechatronics (BioRob). IEEE, 2020.

            c = 0.1
            self.K = np.array( [ [ 29.50, 14.30 ] ,
                                 [ 14.30, 39.30 ] ] )
            self.B = c * self.K

        elif self.n_act == 4:   # The default K and B matrices used for the 2-DOF Robot
                                # [TODO] [2021.07.19] Add reference when it is publicly published
            c = 0.05
            self.K = np.array( [ [ 17.40, 4.70, -1.90, 8.40 ] ,
                                 [  9.00, 33.0,  4.40, 0.00 ] ,
                                 [ -13.6, 3.00,  27.7, 0.00 ] ,
                                 [  8.40, 0.00,  0.00, 23.2 ] ] )
            self.B = c * self.K


        # [2DOF Robot] 5 movement parameters in total - Intial posture (2), Final posture (2) and duration (1)
        # [4DOF RObot] 9 movement parameters in total - Intial posture (4), Final posture (4) and duration (1)
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.3.] Implementation
        # [REF] [Moses C. Nah] Ibid.        ,                                                              [Section 8.2.2.] Zero-Torque Trajectory

        self.n_ctrl_pars    = [ self.n_act ** 2, self.n_act ** 2 ]              # The number of ctrl parameters. This definition would be useful for the optimization process.
                                                                                # K and B has n^2 elements, hence self.n_act ** 2

        self.ctrl_par_names = [  "K", "B" ]                                     # Useful for self.set_ctrl_par method


    def input_calc( self, time ):

        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]
        D  = self.traj.pars[ "D" ]

        if   time <= D:                                                         # If time greater than startTime
            self.x0, self.dx0 = self.traj.func_pos( time ), self.traj.func_vel( time  )  # Calculating the corresponding ZFT of the given time. the startTime should be subtracted for setting the initial time as zero for the ZFT Calculation.
        else:
            self.x0, self.dx0 = self.traj.pars[ "pf" ], np.zeros( ( 4 ) )       # Before start time, the posture should be remained at ZFT's initial posture

        tau_imp = np.dot( self.K, self.x0 - q ) + np.dot( self.B, self.dx0 - dq ) # Calculating the torque due to impedance
        tau_g   = self.get_G( )                                                 # Calculating the torque due to gravity compensation

        return self.mjData.ctrl, self.idx_act, tau_imp + tau_g


class SlidingController( Controller ):
    """
        Description:
        ----------
            Sliding mode controller basic template.
            This will be the parent class for "Joint" and "Cartesian" Space sliding controller.

    """
    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )


    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError( )                                            # Adding this NotImplementedError will force the child class to override parent's methods.
                                                                                # Adding this NotImplementedError will force the child class to override parent's methods.

    def get_M( self, q ):

        M_mat = np.zeros( ( self.n_act, self.n_act ) )

        if  self.n_act == 2:
            # Get the q position, putting the whole
            M1,   M2 = self.M
            Lc1, Lc2 = self.Lc
            L1,   L2 = self.L
            I1xx, I1yy, I1zz = self.I[ 0 ]
            I2xx, I2yy, I2zz = self.I[ 1 ]

            M_mat[ 0, 0 ] = I1yy + I2yy + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 + 2*L1*Lc2*M2*np.cos(q[1])
            M_mat[ 0, 1 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M_mat[ 1, 0 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M_mat[ 1, 1 ] = I2yy + Lc2**2*M2

        elif self.n_act == 4:

            # Get the q position, putting the whole
            M1,   M2 = self.M
            Lc1, Lc2 = self.Lc
            L1,   L2 = self.L
            I1xx, I1yy, I1zz = self.I[ 0 ]
            I2xx, I2yy, I2zz = self.I[ 1 ]

            M_mat[ 0, 0 ] = I1yy + I2yy + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 + 2*L1*Lc2*M2*np.cos(q[1])
            M_mat[ 0, 1 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M_mat[ 1, 0 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M_mat[ 1, 1 ] = I2yy + Lc2**2*M2
            M_mat[ 0, 0 ] = I1zz*np.sin(q[1])**2 + M2*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))**2 + I2xx*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))**2 + I2zz*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3]))**2 + I1xx*np.cos(q[1])**2*np.sin(q[2])**2 + I1yy*np.cos(q[1])**2*np.cos(q[2])**2 + I2yy*np.cos(q[1])**2*np.cos(q[2])**2 + Lc1**2*M1*np.cos(q[1])**2*np.cos(q[2])**2 + Lc1**2*M1*np.cos(q[1])**2*np.sin(q[2])**2 + M2*np.cos(q[1])**2*np.cos(q[2])**2*(Lc2 + L1*np.cos(q[3]))**2 + L1**2*M2*np.cos(q[1])**2*np.cos(q[2])**2*np.sin(q[3])**2
            M_mat[ 0, 1 ] = np.cos(q[2])*(I2zz*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])**2 - I2yy*np.cos(q[1])*np.sin(q[2]) + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) - Lc2**2*M2*np.cos(q[1])*np.sin(q[2]) + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) + L1*Lc2*M2*np.sin(q[1])*np.sin(q[3])) + np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*(I1xx - I1yy)
            M_mat[ 0, 2 ] = - I1zz*np.sin(q[1]) - I2zz*np.cos(q[3])*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3])) - I2xx*np.sin(q[3])*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2])) - Lc2*M2*np.sin(q[3])*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))
            M_mat[ 0, 3 ] = np.cos(q[1])*np.cos(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M_mat[ 1, 0 ] = np.cos(q[2])*(I2zz*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])**2 - I2yy*np.cos(q[1])*np.sin(q[2]) + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) - Lc2**2*M2*np.cos(q[1])*np.sin(q[2]) + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) + L1*Lc2*M2*np.sin(q[1])*np.sin(q[3])) + np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*(I1xx - I1yy)
            M_mat[ 1, 1 ] = I1xx + I2yy - I2yy*np.cos(q[2])**2 + I2zz*np.cos(q[2])**2 + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 - I1xx*np.sin(q[2])**2 + I1yy*np.sin(q[2])**2 - Lc2**2*M2*np.cos(q[2])**2 + I2xx*np.cos(q[2])**2*np.cos(q[3])**2 - I2zz*np.cos(q[2])**2*np.cos(q[3])**2 + 2*L1*Lc2*M2*np.cos(q[3]) + Lc2**2*M2*np.cos(q[2])**2*np.cos(q[3])**2
            M_mat[ 1, 2 ] = -np.cos(q[2])*np.sin(q[3])*(I2xx*np.cos(q[3]) - I2zz*np.cos(q[3]) + L1*Lc2*M2 + Lc2**2*M2*np.cos(q[3]))
            M_mat[ 1, 3 ] = -np.sin(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M_mat[ 2, 0 ] = - I1zz*np.sin(q[1]) - I2zz*np.cos(q[3])*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3])) - I2xx*np.sin(q[3])*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2])) - Lc2*M2*np.sin(q[3])*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))
            M_mat[ 2, 1 ] = -np.cos(q[2])*np.sin(q[3])*(I2xx*np.cos(q[3]) - I2zz*np.cos(q[3]) + L1*Lc2*M2 + Lc2**2*M2*np.cos(q[3]))
            M_mat[ 2, 2 ] = I1zz + I2zz + I2xx*np.sin(q[3])**2 - I2zz*np.sin(q[3])**2 + Lc2**2*M2*np.sin(q[3])**2
            M_mat[ 2, 3 ] = 0
            M_mat[ 3, 0 ] = np.cos(q[1])*np.cos(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M_mat[ 3, 1 ] = -np.sin(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M_mat[ 3, 2 ] = 0
            M_mat[ 3, 3 ] = I2yy + Lc2**2*M2


        return M_mat


    def get_C( self, q, dq):

        C_mat = np.zeros( ( self.n_act, self.n_act ) )

        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        if   self.n_act == 2:

            C_mat[ 0, 0 ] = -L1*Lc2*M2*np.sin(q[1])*dq[1]
            C_mat[ 0, 1 ] = -L1*Lc2*M2*np.sin(q[1])*(dq[0] + dq[1])
            C_mat[ 1, 0 ] = L1*Lc2*M2*np.sin(q[1])*dq[0]
            C_mat[ 1, 1 ] = 0

        elif self.n_act == 4:

            C_mat[0, 0] = (I2xx*np.sin(2*q[1])*dq[1])/2 - (I1xx*np.sin(2*q[1])*dq[1])/2 + (I2xx*np.sin(2*q[3])*dq[3])/2 + (I1zz*np.sin(2*q[1])*dq[1])/2 - (I2zz*np.sin(2*q[1])*dq[1])/2 - (I2zz*np.sin(2*q[3])*dq[3])/2 - (L1**2*M2*np.sin(2*q[1])*dq[1])/2 - (Lc1**2*M1*np.sin(2*q[1])*dq[1])/2 + (Lc2**2*M2*np.sin(2*q[1])*dq[1])/2 + (Lc2**2*M2*np.sin(2*q[3])*dq[3])/2 - I2xx*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] - I2xx*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + I2zz*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] + I2zz*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + I1xx*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] + I1xx*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - I1yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - I2yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - I1yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - I2yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] + I2zz*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] + 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + I2zz*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] + 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[3])*dq[3] + 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] + 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] - 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - Lc2**2*M2*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[3] - Lc2**2*M2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] + I2xx*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + I2xx*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] - I2zz*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - I2zz*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[1] - 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[2] - 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[3] - L1*Lc2*M2*np.sin(q[2])*np.sin(q[3])*dq[1] - 2*L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[1] + I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[1] + Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[3] + 2*L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[1] + 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[3] + 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[2] + L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[3] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2]
            C_mat[0, 1] = (I2xx*np.sin(2*q[1])*dq[0])/2 - (I1xx*np.sin(2*q[1])*dq[0])/2 + (I1zz*np.sin(2*q[1])*dq[0])/2 - (I2zz*np.sin(2*q[1])*dq[0])/2 - (I1xx*np.cos(q[1])*dq[2])/2 - (I2xx*np.cos(q[1])*dq[2])/2 + (I1yy*np.cos(q[1])*dq[2])/2 + (I2yy*np.cos(q[1])*dq[2])/2 - (I1zz*np.cos(q[1])*dq[2])/2 - (I2zz*np.cos(q[1])*dq[2])/2 - (I2xx*np.cos(q[2])*np.sin(q[1])*dq[3])/2 - (I2yy*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (I2zz*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[2] - I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[2] - I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[2] + I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[2] - (L1**2*M2*np.sin(2*q[1])*dq[0])/2 - (Lc1**2*M1*np.sin(2*q[1])*dq[0])/2 + (Lc2**2*M2*np.sin(2*q[1])*dq[0])/2 + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[2] - I1xx*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - I2xx*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + I1yy*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] + I2yy*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - I2zz*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] + I2zz*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + I1xx*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] - I1yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - I2yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] - Lc2**2*M2*np.cos(q[2])*np.sin(q[1])*dq[3] - I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] + 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] - 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[2])*np.sin(q[1])*np.sin(q[2])*dq[1] - Lc2**2*M2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] - I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - L1*Lc2*M2*np.sin(q[2])*np.sin(q[3])*dq[0] - 2*L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[0] + L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[1] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[1] + 2*L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[1] + 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3]
            C_mat[0, 2] = (I1yy*np.cos(q[1])*dq[1])/2 - (I2xx*np.cos(q[1])*dq[1])/2 - (I1xx*np.cos(q[1])*dq[1])/2 + (I2yy*np.cos(q[1])*dq[1])/2 - (I1zz*np.cos(q[1])*dq[1])/2 - (I2zz*np.cos(q[1])*dq[1])/2 + (I2xx*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (I2yy*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (I2zz*np.cos(q[1])*np.sin(q[2])*dq[3])/2 + I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[1] - I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[1] - I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[1] + I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[1] + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[1] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + I1xx*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - I1yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - I2yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + I2zz*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + I2xx*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - I2zz*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[2] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[3] + I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[3])*dq[2] + L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0]
            C_mat[0, 3] = (I2xx*np.sin(2*q[3])*dq[0])/2 - (I2zz*np.sin(2*q[3])*dq[0])/2 - (I2xx*np.cos(q[2])*np.sin(q[1])*dq[1])/2 + (I2xx*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (I2yy*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (I2yy*np.cos(q[1])*np.sin(q[2])*dq[2])/2 + (I2zz*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (I2zz*np.cos(q[1])*np.sin(q[2])*dq[2])/2 + (Lc2**2*M2*np.sin(2*q[3])*dq[0])/2 - I2xx*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + I2zz*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - Lc2**2*M2*np.cos(q[2])*np.sin(q[1])*dq[1] - L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[3])*dq[0] + 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] + I2xx*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[2] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[3] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] + Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1]
            C_mat[1, 0] = (I1xx*np.sin(2*q[1])*dq[0])/2 - (I2xx*np.sin(2*q[1])*dq[0])/2 - (I1zz*np.sin(2*q[1])*dq[0])/2 + (I2zz*np.sin(2*q[1])*dq[0])/2 - (I1xx*np.cos(q[1])*dq[2])/2 + (I2xx*np.cos(q[1])*dq[2])/2 + (I1yy*np.cos(q[1])*dq[2])/2 + (I2yy*np.cos(q[1])*dq[2])/2 + (I1zz*np.cos(q[1])*dq[2])/2 - (I2zz*np.cos(q[1])*dq[2])/2 - (I2xx*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (I2yy*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + (I2zz*np.cos(q[2])*np.sin(q[1])*dq[3])/2 + I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[2] - I2xx*np.cos(q[1])*np.cos(q[3])**2*dq[2] - I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[2] - I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[2] + I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[2] + I2zz*np.cos(q[1])*np.cos(q[3])**2*dq[2] + Lc2**2*M2*np.cos(q[1])*dq[2] + (L1**2*M2*np.sin(2*q[1])*dq[0])/2 + (Lc1**2*M1*np.sin(2*q[1])*dq[0])/2 - (Lc2**2*M2*np.sin(2*q[1])*dq[0])/2 + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[2] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*dq[2] + I2xx*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - I1xx*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + I1yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + I2yy*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] - 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] - 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.sin(q[1])*dq[0] + 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[3] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[2] + L1*Lc2*M2*np.sin(q[2])*np.sin(q[3])*dq[0] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] + 2*L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*dq[0] + L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[3] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*np.sin(q[1])*dq[0] - L1*Lc2*M2*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] - 2*L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2] - 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[3]
            C_mat[1, 1] = - np.sin(q[3])*dq[3]*(L1*Lc2*M2 + I2xx*np.cos(q[2])**2*np.cos(q[3]) - I2zz*np.cos(q[2])**2*np.cos(q[3]) + Lc2**2*M2*np.cos(q[2])**2*np.cos(q[3])) - np.cos(q[2])*np.sin(q[2])*dq[2]*(I1xx + I2xx/2 - I1yy - I2yy + I2zz/2 + (I2xx*(2*np.cos(q[3])**2 - 1))/2 - (I2zz*(2*np.cos(q[3])**2 - 1))/2 - (Lc2**2*M2)/2 + (Lc2**2*M2*(2*np.cos(q[3])**2 - 1))/2)
            C_mat[1, 2] = (I1yy*np.sin(2*q[2])*dq[1])/2 - (I1xx*np.sin(2*q[2])*dq[1])/2 + (I2yy*np.sin(2*q[2])*dq[1])/2 - (I2zz*np.sin(2*q[2])*dq[1])/2 - (I1xx*np.cos(q[1])*dq[0])/2 + (I2xx*np.cos(q[1])*dq[0])/2 + (I2xx*np.cos(q[2])*dq[3])/2 + (I1yy*np.cos(q[1])*dq[0])/2 + (I2yy*np.cos(q[1])*dq[0])/2 - (I2yy*np.cos(q[2])*dq[3])/2 + (I1zz*np.cos(q[1])*dq[0])/2 - (I2zz*np.cos(q[1])*dq[0])/2 - (I2zz*np.cos(q[2])*dq[3])/2 + I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[0] - I2xx*np.cos(q[1])*np.cos(q[3])**2*dq[0] - I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[3] - I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[0] - I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[0] + I2zz*np.cos(q[1])*np.cos(q[3])**2*dq[0] + I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[3] + Lc2**2*M2*np.cos(q[1])*dq[0] + (Lc2**2*M2*np.sin(2*q[2])*dq[1])/2 + I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*dq[0] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[3] + I2xx*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] - I2zz*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] - I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] + I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] + Lc2**2*M2*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[2] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*dq[3] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + L1*Lc2*M2*np.sin(q[2])*np.sin(q[3])*dq[2] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - L1*Lc2*M2*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0]
            C_mat[1, 3] = (I2xx*np.cos(q[2])*dq[2])/2 - (I2yy*np.cos(q[2])*dq[2])/2 - (I2zz*np.cos(q[2])*dq[2])/2 - (I2xx*np.cos(q[2])*np.sin(q[1])*dq[0])/2 + (I2yy*np.cos(q[2])*np.sin(q[1])*dq[0])/2 + (I2zz*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[2] + I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[2] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[2] - L1*Lc2*M2*np.sin(q[3])*dq[1] + I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - I2xx*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + I2zz*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - Lc2**2*M2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*dq[2] + L1*Lc2*M2*np.sin(q[2])*np.sin(q[3])*dq[3] + L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]
            C_mat[2, 0] = (I1xx*np.cos(q[1])*dq[1])/2 - (I2xx*np.cos(q[1])*dq[1])/2 - (I1yy*np.cos(q[1])*dq[1])/2 - (I2yy*np.cos(q[1])*dq[1])/2 - (I1zz*np.cos(q[1])*dq[1])/2 + (I2zz*np.cos(q[1])*dq[1])/2 + (I2xx*np.cos(q[1])*np.sin(q[2])*dq[3])/2 + (I2yy*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - (I2zz*np.cos(q[1])*np.sin(q[2])*dq[3])/2 - I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[1] + I2xx*np.cos(q[1])*np.cos(q[3])**2*dq[1] + I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[1] + I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[1] - I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[1] - I2zz*np.cos(q[1])*np.cos(q[3])**2*dq[1] - Lc2**2*M2*np.cos(q[1])*dq[1] - I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[1] + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*dq[1] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - I1xx*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] + I1yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + I2yy*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - I2zz*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] + Lc2**2*M2*np.cos(q[1])*np.sin(q[2])*dq[3] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[3] - I2xx*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + I2zz*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[3] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[1] + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] - I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + L1*Lc2*M2*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0]
            C_mat[2, 1] = (I1xx*np.sin(2*q[2])*dq[1])/2 - (I1yy*np.sin(2*q[2])*dq[1])/2 - (I2yy*np.sin(2*q[2])*dq[1])/2 + (I2zz*np.sin(2*q[2])*dq[1])/2 + (I1xx*np.cos(q[1])*dq[0])/2 - (I2xx*np.cos(q[1])*dq[0])/2 + (I2xx*np.cos(q[2])*dq[3])/2 - (I1yy*np.cos(q[1])*dq[0])/2 - (I2yy*np.cos(q[1])*dq[0])/2 + (I2yy*np.cos(q[2])*dq[3])/2 - (I1zz*np.cos(q[1])*dq[0])/2 + (I2zz*np.cos(q[1])*dq[0])/2 - (I2zz*np.cos(q[2])*dq[3])/2 - I1xx*np.cos(q[1])*np.cos(q[2])**2*dq[0] + I2xx*np.cos(q[1])*np.cos(q[3])**2*dq[0] - I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[3] + I1yy*np.cos(q[1])*np.cos(q[2])**2*dq[0] + I2yy*np.cos(q[1])*np.cos(q[2])**2*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])**2*dq[0] - I2zz*np.cos(q[1])*np.cos(q[3])**2*dq[0] + I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[3] - Lc2**2*M2*np.cos(q[1])*dq[0] + Lc2**2*M2*np.cos(q[2])*dq[3] - (Lc2**2*M2*np.sin(2*q[2])*dq[1])/2 - I2xx*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + I2zz*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*dq[0] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[3] + I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[2])*dq[1] - Lc2**2*M2*np.cos(q[1])*np.cos(q[2])**2*np.cos(q[3])**2*dq[0] + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + L1*Lc2*M2*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0]
            C_mat[2, 2] = np.sin(2*q[3])*dq[3]*(I2xx/2 - I2zz/2 + (Lc2**2*M2)/2)
            C_mat[2, 3] = (I2xx*np.sin(2*q[3])*dq[2])/2 - (I2zz*np.sin(2*q[3])*dq[2])/2 + (I2xx*np.cos(q[2])*dq[1])/2 + (I2yy*np.cos(q[2])*dq[1])/2 - (I2zz*np.cos(q[2])*dq[1])/2 + (I2xx*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + (I2yy*np.cos(q[1])*np.sin(q[2])*dq[0])/2 - (I2zz*np.cos(q[1])*np.sin(q[2])*dq[0])/2 - I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[1] + I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[1] + Lc2**2*M2*np.cos(q[2])*dq[1] + (Lc2**2*M2*np.sin(2*q[3])*dq[2])/2 - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[1] - I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0]
            C_mat[3, 0] = (I2zz*np.sin(2*q[3])*dq[0])/2 - (I2xx*np.sin(2*q[3])*dq[0])/2 + (I2xx*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (I2xx*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (I2yy*np.cos(q[2])*np.sin(q[1])*dq[1])/2 - (I2yy*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (I2zz*np.cos(q[2])*np.sin(q[1])*dq[1])/2 + (I2zz*np.cos(q[1])*np.sin(q[2])*dq[2])/2 - (Lc2**2*M2*np.sin(2*q[3])*dq[0])/2 + I2xx*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - I2zz*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*I2xx*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] + I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] - 2*I2zz*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - Lc2**2*M2*np.cos(q[1])*np.sin(q[2])*dq[2] + L1*Lc2*M2*np.cos(q[1])**2*np.sin(q[3])*dq[0] - 2*I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + 2*I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.sin(q[1])*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[2] - I2xx*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + I2zz*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[1] + 2*Lc2**2*M2*np.cos(q[1])**2*np.cos(q[3])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[2] - L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[1] + I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1] - Lc2**2*M2*np.cos(q[1])**2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[0] - 2*Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[1])*np.sin(q[2])*dq[0] - L1*Lc2*M2*np.cos(q[1])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[1]
            C_mat[3, 1] = (I2zz*np.cos(q[2])*dq[2])/2 - (I2yy*np.cos(q[2])*dq[2])/2 - (I2xx*np.cos(q[2])*dq[2])/2 + (I2xx*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - (I2yy*np.cos(q[2])*np.sin(q[1])*dq[0])/2 - (I2zz*np.cos(q[2])*np.sin(q[1])*dq[0])/2 + I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[2] - I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[2] - Lc2**2*M2*np.cos(q[2])*dq[2] + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[2] + L1*Lc2*M2*np.sin(q[3])*dq[1] - I2xx*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + I2xx*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] + I2zz*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] - I2zz*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*np.sin(q[1])*dq[0] + Lc2**2*M2*np.cos(q[2])**2*np.cos(q[3])*np.sin(q[3])*dq[1] - L1*Lc2*M2*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[0] + I2xx*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*np.sin(q[2])*np.sin(q[3])*dq[0]
            C_mat[3, 2] = (I2zz*np.sin(2*q[3])*dq[2])/2 - (I2xx*np.sin(2*q[3])*dq[2])/2 - (I2xx*np.cos(q[2])*dq[1])/2 - (I2yy*np.cos(q[2])*dq[1])/2 + (I2zz*np.cos(q[2])*dq[1])/2 - (I2xx*np.cos(q[1])*np.sin(q[2])*dq[0])/2 - (I2yy*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + (I2zz*np.cos(q[1])*np.sin(q[2])*dq[0])/2 + I2xx*np.cos(q[2])*np.cos(q[3])**2*dq[1] - I2zz*np.cos(q[2])*np.cos(q[3])**2*dq[1] - Lc2**2*M2*np.cos(q[2])*dq[1] - (Lc2**2*M2*np.sin(2*q[3])*dq[2])/2 + Lc2**2*M2*np.cos(q[2])*np.cos(q[3])**2*dq[1] + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - I2zz*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0] - Lc2**2*M2*np.cos(q[1])*np.sin(q[2])*dq[0] + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3])*dq[0] + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2])*dq[0]
            C_mat[3, 3] = 0

        return C_mat


class JointSlidingController( SlidingController ):

    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )

        self.n_ctrl_pars    = [ self.n_act ** 2, self.n_act ** 2 ]              # The number of ctrl parameters.
        self.ctrl_par_names = [ "Kd", "Kl" ]                                    # Kl: s = q' + Kl q
                                                                                # Kd: tau = M(q)qr'' + C(q,q')qr' + G(q) - Kds


    def input_calc( self, time ):

        q    = self.mjData.qpos[ 0 : self.n_act ]
        dq   = self.mjData.qvel[ 0 : self.n_act ]

        if   time <= self.traj.pars[ "D" ]:
            self.qd   = self.traj.func_pos( time )
            self.dqd  = self.traj.func_vel( time )
            self.ddqd = self.traj.func_acc( time )

        else:
            self.qd   = np.array( self.traj.pars[ "pf" ] )
            self.dqd  = np.zeros( self.n_act )
            self.ddqd = np.zeros( self.n_act )

        dqr    =  self.dqd - self.Kl.dot(  q - self.qd   )
        ddqr   = self.ddqd - self.Kl.dot( dq - self.dqd  )
        self.s = dq - dqr

        Mmat = self.get_M( q     )
        Cmat = self.get_C( q, dq )
        Gmat = self.get_G(       )

        tau = Mmat.dot( ddqr ) + Cmat.dot( dqr ) + Gmat - self.Kd.dot( self.s )

        return self.mjData.ctrl, self.idx_act, tau


class CartesianImpedanceController( ImpedanceController ):
    """
        Description:
        ----------
            Controller for xyz coordinate.

    """
    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )

        if   self.n_act == 2:     # If 2D Model
            self.n_mov_pars     = 5                                             # Starting point (2), ending point (2) and the duration (1) between the two.
            self.mov_parameters = None                                          # The actual values of the movement parameters, initializing it with random values
            self.n_ctrl_pars    = [ self.n_mov_pars, 2 ** 2, 2 ** 2 ]           # The number of ctrl parameters. This definition would be useful for the optimization process.
                                                                                # K and B has 2^2 elements, hence 4

        elif self.n_act == 4:
            self.n_mov_pars     = 7                                             # Starting point (2), ending point (2) and the duration (1) between the two.
            self.mov_parameters = None                                          # The actual values of the movement parameters, initializing it with random values
            self.n_ctrl_pars    = [ self.n_mov_pars, 3 ** 2, 3 ** 2 ]           # The number of ctrl parameters. This definition would be useful for the optimization process.
                                                                                # K and B has 2^2 elements, hence 4

        self.ctrl_par_names = [ "mov_parameters", "K", "B" ]                    # Useful for self.set_ctrl_par method


    def get_dJ( self ):
        # Get the time derivative of the Jacobian matrix. dJ/dt

        L1, L2 = self.L
        q      = self.mjData.qpos[ 0 : self.n_act ]
        dq     = self.mjData.qvel[ 0 : self.n_act ]

        if self.n_act == 2:

            dJ = np.zeros( (2, 2) )  # Since the y direction doesn't matter.

            dJ[0, 0] = - L1*np.sin(q[0])*dq[0] - L2*np.cos(q[0])*np.sin(q[1])*dq[0] - L2*np.cos(q[1])*np.sin(q[0])*dq[0] - L2*np.cos(q[0])*np.sin(q[1])*dq[1] - L2*np.cos(q[1])*np.sin(q[0])*dq[1]
            dJ[0, 1] = -L2*(np.cos(q[0])*np.sin(q[1])*dq[0] + np.cos(q[1])*np.sin(q[0])*dq[0] + np.cos(q[0])*np.sin(q[1])*dq[1] + np.cos(q[1])*np.sin(q[0])*dq[1])
            dJ[1, 0] = L1*np.cos(q[0])*dq[0] + L2*np.cos(q[0])*np.cos(q[1])*dq[0] + L2*np.cos(q[0])*np.cos(q[1])*dq[1] - L2*np.sin(q[0])*np.sin(q[1])*dq[0] - L2*np.sin(q[0])*np.sin(q[1])*dq[1]
            dJ[1, 1] = L2*(np.cos(q[0])*np.cos(q[1])*dq[0] + np.cos(q[0])*np.cos(q[1])*dq[1] - np.sin(q[0])*np.sin(q[1])*dq[0] - np.sin(q[0])*np.sin(q[1])*dq[1])

        elif self.n_act == 4:

            dJ = np.zeros( (3, 4) )

            dJ[0, 0] = L2*np.sin(q[0])*np.sin(q[2])*np.sin(q[3])*dq[2] - L1*np.cos(q[0])*np.sin(q[1])*dq[1] - L2*np.cos(q[1])*np.cos(q[3])*np.sin(q[0])*dq[0] - L2*np.cos(q[0])*np.cos(q[2])*np.sin(q[3])*dq[0] - L2*np.cos(q[0])*np.cos(q[3])*np.sin(q[1])*dq[1] - L2*np.cos(q[0])*np.cos(q[1])*np.sin(q[3])*dq[3] - L2*np.cos(q[2])*np.cos(q[3])*np.sin(q[0])*dq[3] - L1*np.cos(q[1])*np.sin(q[0])*dq[0] + L2*np.cos(q[0])*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] + L2*np.cos(q[0])*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[2] + L2*np.cos(q[0])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[3] - L2*np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0]
            dJ[0, 1] = L2*np.sin(q[0])*np.sin(q[1])*np.sin(q[3])*dq[3] - L1*np.cos(q[1])*np.sin(q[0])*dq[1] - L2*np.cos(q[0])*np.cos(q[3])*np.sin(q[1])*dq[0] - L2*np.cos(q[1])*np.cos(q[3])*np.sin(q[0])*dq[1] - L1*np.cos(q[0])*np.sin(q[1])*dq[0] + L2*np.cos(q[0])*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + L2*np.cos(q[1])*np.cos(q[2])*np.sin(q[0])*np.sin(q[3])*dq[2] + L2*np.cos(q[1])*np.cos(q[3])*np.sin(q[0])*np.sin(q[2])*dq[3] - L2*np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1]
            dJ[0, 2] = L2*np.sin(q[3])*(np.sin(q[0])*np.sin(q[2])*dq[0] - np.cos(q[0])*np.cos(q[2])*dq[2] + np.cos(q[0])*np.cos(q[2])*np.sin(q[1])*dq[0] + np.cos(q[1])*np.cos(q[2])*np.sin(q[0])*dq[1] - np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*dq[2]) - L2*np.cos(q[3])*dq[3]*(np.cos(q[0])*np.sin(q[2]) - np.cos(q[2])*np.sin(q[0])*np.sin(q[1]))
            dJ[0, 3] = -L2*(np.cos(q[0])*np.cos(q[1])*np.sin(q[3])*dq[0] + np.cos(q[2])*np.cos(q[3])*np.sin(q[0])*dq[0] + np.cos(q[0])*np.cos(q[3])*np.sin(q[2])*dq[2] + np.cos(q[1])*np.cos(q[3])*np.sin(q[0])*dq[3] + np.cos(q[0])*np.cos(q[2])*np.sin(q[3])*dq[3] - np.sin(q[0])*np.sin(q[1])*np.sin(q[3])*dq[1] - np.cos(q[0])*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[0] - np.cos(q[1])*np.cos(q[3])*np.sin(q[0])*np.sin(q[2])*dq[1] - np.cos(q[2])*np.cos(q[3])*np.sin(q[0])*np.sin(q[1])*dq[2] + np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[3])
            dJ[1, 0] = 0
            dJ[1, 1] = L1*np.sin(q[1])*dq[1] + L2*np.cos(q[3])*np.sin(q[1])*dq[1] + L2*np.cos(q[1])*np.sin(q[3])*dq[3] - L2*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1] - L2*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[2] - L2*np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[3]
            dJ[1, 2] = L2*np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*dq[3] - L2*np.cos(q[2])*np.sin(q[1])*np.sin(q[3])*dq[1] - L2*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])*dq[2]
            dJ[1, 3] = L2*(np.cos(q[1])*np.sin(q[3])*dq[1] + np.cos(q[3])*np.sin(q[1])*dq[3] + np.cos(q[1])*np.cos(q[2])*np.cos(q[3])*dq[2] - np.cos(q[3])*np.sin(q[1])*np.sin(q[2])*dq[1] - np.cos(q[1])*np.sin(q[2])*np.sin(q[3])*dq[3])
            dJ[2, 0] = L1*np.cos(q[0])*np.cos(q[1])*dq[0] - L1*np.sin(q[0])*np.sin(q[1])*dq[1] + L2*np.cos(q[0])*np.cos(q[1])*np.cos(q[3])*dq[0] + L2*np.cos(q[0])*np.cos(q[2])*np.cos(q[3])*dq[3] - L2*np.cos(q[2])*np.sin(q[0])*np.sin(q[3])*dq[0] - L2*np.cos(q[3])*np.sin(q[0])*np.sin(q[1])*dq[1] - L2*np.cos(q[0])*np.sin(q[2])*np.sin(q[3])*dq[2] - L2*np.cos(q[1])*np.sin(q[0])*np.sin(q[3])*dq[3] + L2*np.cos(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[0] + L2*np.cos(q[1])*np.sin(q[0])*np.sin(q[2])*np.sin(q[3])*dq[1] + L2*np.cos(q[2])*np.sin(q[0])*np.sin(q[1])*np.sin(q[3])*dq[2] + L2*np.cos(q[3])*np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*dq[3]
            dJ[2, 1] = L1*np.cos(q[0])*np.cos(q[1])*dq[1] - L1*np.sin(q[0])*np.sin(q[1])*dq[0] + L2*np.cos(q[0])*np.cos(q[1])*np.cos(q[3])*dq[1] - L2*np.cos(q[3])*np.sin(q[0])*np.sin(q[1])*dq[0] - L2*np.cos(q[0])*np.sin(q[1])*np.sin(q[3])*dq[3] - L2*np.cos(q[0])*np.cos(q[1])*np.cos(q[2])*np.sin(q[3])*dq[2] - L2*np.cos(q[0])*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[3] + L2*np.cos(q[1])*np.sin(q[0])*np.sin(q[2])*np.sin(q[3])*dq[0] + L2*np.cos(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[1]
            dJ[2, 2] = - L2*np.sin(q[3])*(np.cos(q[0])*np.sin(q[2])*dq[0] + np.cos(q[2])*np.sin(q[0])*dq[2] + np.cos(q[0])*np.cos(q[1])*np.cos(q[2])*dq[1] - np.cos(q[2])*np.sin(q[0])*np.sin(q[1])*dq[0] - np.cos(q[0])*np.sin(q[1])*np.sin(q[2])*dq[2]) - L2*np.cos(q[3])*dq[3]*(np.sin(q[0])*np.sin(q[2]) + np.cos(q[0])*np.cos(q[2])*np.sin(q[1]))
            dJ[2, 3] = -L2*(np.cos(q[1])*np.sin(q[0])*np.sin(q[3])*dq[0] - np.cos(q[0])*np.cos(q[1])*np.cos(q[3])*dq[3] - np.cos(q[0])*np.cos(q[2])*np.cos(q[3])*dq[0] + np.cos(q[0])*np.sin(q[1])*np.sin(q[3])*dq[1] + np.cos(q[3])*np.sin(q[0])*np.sin(q[2])*dq[2] + np.cos(q[2])*np.sin(q[0])*np.sin(q[3])*dq[3] + np.cos(q[0])*np.cos(q[1])*np.cos(q[3])*np.sin(q[2])*dq[1] + np.cos(q[0])*np.cos(q[2])*np.cos(q[3])*np.sin(q[1])*dq[2] - np.cos(q[3])*np.sin(q[0])*np.sin(q[1])*np.sin(q[2])*dq[0] - np.cos(q[0])*np.sin(q[1])*np.sin(q[2])*np.sin(q[3])*dq[3])

        return dJ





    def set_ZFT( self, mov_parameters = None ):
        """
            Description:
            ----------
                Setting the ZFT(Zero-torque trajectory) of the Cartesian Impedance Controller.
                This method is only called once "before" running the simulation, and "after" the self.mov_parameters are well-defined
                This is for

        """

        if mov_parameters is not None:
            self.mov_parameters = mov_parameters

        # Defining the equations for the ZFT, this function must be done before the actual simulation
        if self.mov_parameters is None:
            raise ValueError( "Movement parameters are not defined")

        xi = np.array( self.mov_parameters[           0 :     self.n_act ] )    # Initial Cartesian Posture  of the end-effector
        xf = np.array( self.mov_parameters[  self.n_act : 2 * self.n_act ] )    # Final Cartesian Position of the end-effector
        D  = self.mov_parameters[ -1 ]                                          # Duration it took from start to end

        # Basis function used for the ZFT Trajectory is minimum-jerk-trajectory
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.2.] Zero-torque trajectory
        # [REF] [T. Flash and N. Hogan]             : "Flash, Tamar, and Neville Hogan. "The coordination of arm movements: an experimentally confirmed mathematical model."
        self.ZFT_func_pos = min_jerk_traj( self.t_sym, xi,xf, D )
        self.ZFT_func_vel = [ sp.diff( tmp, self.t_sym ) for tmp in self.ZFT_func_pos ]

        # Lambdify the functions
        # [TIP] This is necessary for computation Speed!
        self.ZFT_func_pos = lambdify( self.t_sym, self.ZFT_func_pos )
        self.ZFT_func_vel = lambdify( self.t_sym, self.ZFT_func_vel )

    def get_ZFT( self, time ):

        D = self.mov_parameters[ -1 ]                                           # Last element is duration
        t = D if time >= D else time                                            # Rectifying the time value if time is larger than D
                                                                                # This means that the ZFT of the controller remains at final posture.
        x0  = np.array( self.ZFT_func_pos( t ) )
        dx0 = np.array( self.ZFT_func_vel( t ) )


        return x0, dx0

    def input_calc( self, start_time, current_time ):

        # The detailed equation of the movement is as follows:
        JEE  = self.mjData.get_geom_jacp( "geom_EE" ).reshape( 3, -1 )[ :, 0 : self.n_act ]  # Get the end-effector Jacobian


        if   self.n_act == 2:
            # If 2DOF robot model, then no movement in the y-direction since the movement is confined to the 2D sagittal plane
            JEE     = np.delete( JEE , 1, 0 )         # Erasing the element (2nd row) to prevent singularity

        dJ   = self.get_dJ( )
        Mq   = self.get_M( )
        C    = self.get_C( )
        G    = self.get_G( )

        Mx   = np.linalg.inv( JEE.dot(  np.linalg.inv( Mq ) ).dot( JEE.T  ) )

        self.Mmat   = np.linalg.inv( JEE.dot(  np.linalg.inv( Mq ) ).dot( JEE.T  ) )
        self.dJ     = dJ

        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]

        x  = self.mjData.get_geom_xpos( "geom_EE" )

        if   self.n_act == 2:
            # If 2DOF robot model, then no movement in the y-direction since the movement is confined to the 2D sagittal plane
            x     = np.delete( x , 1, 0 )         # Erasing the element (2nd row) to prevent singularity


        dx = JEE.dot( dq )

        A = dJ.dot( dq ) - JEE.dot( np.linalg.inv( Mq ) ).dot( C ).dot( dq ) - JEE.dot( np.linalg.inv( Mq )  ).dot( G )     # The nonlinear terms

        Kx = 5 * np.eye( self.n_act )
        Bx = 1 * Kx


        if   current_time >= start_time:                                        # If time greater than startTime
            self.x0, self.dx0 = self.get_ZFT( current_time - start_time  )      # Calculating the corresponding ZFT of the given time. the startTime should be subtracted for setting the initial time as zero for the ZFT Calculation.
        else:
            self.x0, self.dx0 = self.get_ZFT( 0  )                              # Before start time, the posture should be remained at ZFT's initial posture


        # print( np.amax( Mx ) )
        # [Moses C. Nah]
        # Usually, if the condition number of the Jacobian is high (i.e., the jacobian matrix is being singuler
        # Then the dJ term explodes since Mx contains a jacobian inverse
        # Hence, we need to check the jacobian matrix to make sure the simulation doesn't blow up.
        # The quick way to fix this is to modify JEE so that it avoids singularity

        # [TODO] Just neglecting the dJ term.
        # if  np.linalg.cond( JEE ) > 30: # threshold value is 50 for this case.
        #     # In case you want to inverse the matrix
        #     # [BACKUP] [MOSES C NAH]
        #     Mx = np.linalg.pinv( JEE.dot(  np.linalg.inv( Mq ) ).dot( JEE.T  ) )        # [REF] https://stackoverflow.com/questions/49357417/why-is-numpy-linalg-pinv-preferred-over-numpy-linalg-inv-for-creating-invers
        #                                                                                 # [REF] https://en.wikipedia.org/wiki/Moore%E2%80%93Penrose_inverse
        #     A = np.zeros( self.n_act )
        #
        # else:
        #     A = JEE.dot( Mx.dot( dJ.dot( dq ) )  )
        #
        # # If still A value is so high, then just set as zero
        # print( A )

        tau = JEE.T.dot( Bx.dot( self.dx0 - dx ) + Kx.dot( self.x0 - x ) ) + C.dot( dq ) + G #A

        return self.mjData.ctrl, self.idx_act, tau



if __name__ == "__main__":
    pass

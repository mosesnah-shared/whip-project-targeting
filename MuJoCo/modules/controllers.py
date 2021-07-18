# [Built-in modules]

# [3rd party modules]
import numpy as np
import sys
import time
import pickle

from   modules.utils        import my_print
from   modules.traj_funcs   import min_jerk_traj
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


    def __init__( self, mjModel, mjData, mjArgs ):
        """

        """
        self.mjModel        = mjModel
        self.mjData         = mjData
        self.mjArgs         = mjArgs
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


class NullController( Controller ):
    """
        Description:
        ----------
            Controller which is simply empty, useful when practicing/debugging with MuJoCo

    """
    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )
        self.n_act = 0

    def set_ZFT( self ):
        return 0

    def input_calc( self, start_time, current_time ):
        return None, None, 0


# [TODO] [Moses C. Nah] [04.20.2021]
# We can simply use inheritance for the impedance controller

class ImpedanceController( Controller ):
    """
        Description:
        ----------
            Class for an Impedance Controller
            Inheritance of parent class "Contronller"

    """
    def __init__( self, mjModel, mjData, mjArgs ):

        super().__init__( mjModel, mjData, mjArgs )

        self.act_names      = mjModel.actuator_names                            # The names of the actuators, all the names end with "TorqueMotor" (Refer to xml model files)
        self.n_act          = len( mjModel.actuator_names )                     # The number of actuators, 2 for 2D model and 4 for 3D model
        self.idx_act        = np.arange( 0, self.n_act )                        # The idx array of the actuators, this is useful for self.input_calc method
        self.n_limbs        = '-'.join( mjModel.body_names ).lower().count( 'arm' ) # The number of limbs of the controller. Checking bodies which contain "arm" as the name (Refer to xml model files)
        self.type           = None
        self.g              = mjModel.opt.gravity                               # The gravity vector of the simulation

        # Impedance Controller uses pos, vel and acc of the ZFT (Zero-force trajectory), ZTT (Zero-torque trajectory)
        self.ZFT_func_pos   = None
        self.ZFT_func_vel   = None
        self.ZFT_func_acc   = None

        if self.n_limbs == 2:                                                   # For arm model with 2 limbs.
            bodyName  = ['upperArm', 'foreArm' ]                                # Masses of the body that are needed for the gravity compensation

            # Mass and Inertia Information of the limbs.
            self.M  = [ self.mjModel.body_mass[ idx ]     for idx, s in enumerate( self.mjModel.body_names ) if s in bodyName ]
            self.I  = [ self.mjModel.body_inertia[ idx ]  for idx, s in enumerate( self.mjModel.body_names ) if s in bodyName ]

            # The length of the limb and length from proximal joint to center of mass
            self.L  = [ abs( self.mjData.get_geom_xpos( "elbowGEOM"   )[ 2 ] ), abs( self.mjData.get_geom_xpos( "geom_EE"    )[ 2 ] ) - abs( self.mjData.get_geom_xpos( "elbowGEOM"   )[ 2 ] ) ]
            self.Lc = [ abs( self.mjData.get_site_xpos( "upperArmCOM" )[ 2 ] ), abs( self.mjData.get_site_xpos( "foreArmCOM" )[ 2 ] ) - abs( self.mjData.get_geom_xpos( "elbowGEOM"   )[ 2 ] ) ]

            # The mass of the whip is the total mass
            self.Mw = sum( self.mjModel.body_mass[ : ] ) - sum( self.M )

        elif self.n_limbs == 3:
            raise NotImplementedError( )

        # The impedance parameter of the controller
        self.Kmat = None
        self.Bmat = None
        self.Mmat = None

        self.n_mov_pars     = None                                              # The number of parameters of the movement
        self.mov_parameters = None                                              # The actual values of the movement parameters, initializing it with random values
        self.n_ctrl_pars    = None                                              # The number of ctrl parameters. This definition would be useful for the optimization process.
        self.ctrl_par_names = None                                              # Useful for self.set_ctrl_par method
        self.t_sym = sp.symbols( 't' )                                          # time symbol of the equation


    def input_calc( self, start_time, current_time ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.

    def set_ZFT( self ):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.

    def get_ZFT( self):
        """
            Calculating the torque input
        """
        raise NotImplementedError                                               # Adding this NotImplementedError will force the child class to override parent's methods.


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


    def get_C( self ):

        # Just to simplfy or shrinken the length of code for this.
        q        = self.mjData.qpos[ 0 : self.n_act ]
        dq       = self.mjData.qvel[ 0 : self.n_act ]
        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        if   self.n_act == 2:

            C = np.zeros( (2, 2) )

            C[0, 0] = -L1*Lc2*M2*np.sin(q[1])*dq[1]
            C[0, 1] = -L1*Lc2*M2*np.sin(q[1])*(dq[0] + dq[1])
            C[1, 0] = L1*Lc2*M2*np.sin(q[1])*dq[0]
            C[1, 1] = 0

        elif self.n_act == 4:
            NotImplementedError( )

        return C


    def get_M( self ):

        # Just to simplfy or shrinken the length of code for this.
        q        = self.mjData.qpos[ 0 : self.n_act ]
        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        if   self.n_act == 2:

            M = np.zeros( (2, 2) )

            M[ 0, 0 ] = I1yy + I2yy + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 + 2*L1*Lc2*M2*np.cos(q[1])
            M[ 0, 1 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M[ 1, 0 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M[ 1, 1 ] = I2yy + Lc2**2*M2

        elif self.n_act == 4:

            M = np.zeros( (4, 4) )

            M[ 0, 0 ] = I1yy + I2yy + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 + 2*L1*Lc2*M2*np.cos(q[1])
            M[ 0, 1 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M[ 1, 0 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
            M[ 1, 1 ] = I2yy + Lc2**2*M2
            M[ 0, 0 ] = I1zz*np.sin(q[1])**2 + M2*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))**2 + I2xx*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))**2 + I2zz*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3]))**2 + I1xx*np.cos(q[1])**2*np.sin(q[2])**2 + I1yy*np.cos(q[1])**2*np.cos(q[2])**2 + I2yy*np.cos(q[1])**2*np.cos(q[2])**2 + Lc1**2*M1*np.cos(q[1])**2*np.cos(q[2])**2 + Lc1**2*M1*np.cos(q[1])**2*np.sin(q[2])**2 + M2*np.cos(q[1])**2*np.cos(q[2])**2*(Lc2 + L1*np.cos(q[3]))**2 + L1**2*M2*np.cos(q[1])**2*np.cos(q[2])**2*np.sin(q[3])**2
            M[ 0, 1 ] = np.cos(q[2])*(I2zz*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])**2 - I2yy*np.cos(q[1])*np.sin(q[2]) + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) - Lc2**2*M2*np.cos(q[1])*np.sin(q[2]) + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) + L1*Lc2*M2*np.sin(q[1])*np.sin(q[3])) + np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*(I1xx - I1yy)
            M[ 0, 2 ] = - I1zz*np.sin(q[1]) - I2zz*np.cos(q[3])*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3])) - I2xx*np.sin(q[3])*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2])) - Lc2*M2*np.sin(q[3])*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))
            M[ 0, 3 ] = np.cos(q[1])*np.cos(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M[ 1, 0 ] = np.cos(q[2])*(I2zz*np.cos(q[1])*np.sin(q[2])*np.sin(q[3])**2 - I2yy*np.cos(q[1])*np.sin(q[2]) + I2xx*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) - I2zz*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + I2xx*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) - Lc2**2*M2*np.cos(q[1])*np.sin(q[2]) + Lc2**2*M2*np.cos(q[3])*np.sin(q[1])*np.sin(q[3]) + Lc2**2*M2*np.cos(q[1])*np.cos(q[3])**2*np.sin(q[2]) + L1*Lc2*M2*np.sin(q[1])*np.sin(q[3])) + np.cos(q[1])*np.cos(q[2])*np.sin(q[2])*(I1xx - I1yy)
            M[ 1, 1 ] = I1xx + I2yy - I2yy*np.cos(q[2])**2 + I2zz*np.cos(q[2])**2 + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 - I1xx*np.sin(q[2])**2 + I1yy*np.sin(q[2])**2 - Lc2**2*M2*np.cos(q[2])**2 + I2xx*np.cos(q[2])**2*np.cos(q[3])**2 - I2zz*np.cos(q[2])**2*np.cos(q[3])**2 + 2*L1*Lc2*M2*np.cos(q[3]) + Lc2**2*M2*np.cos(q[2])**2*np.cos(q[3])**2
            M[ 1, 2 ] = -np.cos(q[2])*np.sin(q[3])*(I2xx*np.cos(q[3]) - I2zz*np.cos(q[3]) + L1*Lc2*M2 + Lc2**2*M2*np.cos(q[3]))
            M[ 1, 3 ] = -np.sin(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M[ 2, 0 ] = - I1zz*np.sin(q[1]) - I2zz*np.cos(q[3])*(np.cos(q[3])*np.sin(q[1]) - np.cos(q[1])*np.sin(q[2])*np.sin(q[3])) - I2xx*np.sin(q[3])*(np.sin(q[1])*np.sin(q[3]) + np.cos(q[1])*np.cos(q[3])*np.sin(q[2])) - Lc2*M2*np.sin(q[3])*(L1*np.cos(q[1])*np.sin(q[2]) + Lc2*np.sin(q[1])*np.sin(q[3]) + Lc2*np.cos(q[1])*np.cos(q[3])*np.sin(q[2]))
            M[ 2, 1 ] = -np.cos(q[2])*np.sin(q[3])*(I2xx*np.cos(q[3]) - I2zz*np.cos(q[3]) + L1*Lc2*M2 + Lc2**2*M2*np.cos(q[3]))
            M[ 2, 2 ] = I1zz + I2zz + I2xx*np.sin(q[3])**2 - I2zz*np.sin(q[3])**2 + Lc2**2*M2*np.sin(q[3])**2
            M[ 2, 3 ] = 0
            M[ 3, 0 ] = np.cos(q[1])*np.cos(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M[ 3, 1 ] = -np.sin(q[2])*(I2yy + Lc2**2*M2 + L1*Lc2*M2*np.cos(q[3]))
            M[ 3, 2 ] = 0
            M[ 3, 3 ] = I2yy + Lc2**2*M2

        return M


    def get_G( self ):

        if   self.n_limbs == 2:

            # Torque for Gravity compensation is simply tau = J^TF
            # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
            G = np.dot( self.mjData.get_site_jacp( "upperArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[0] * self.g  )  \
              + np.dot( self.mjData.get_site_jacp(  "foreArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[1] * self.g  )

            if "_w_" in self.mjArgs[ 'modelName' ]: # If a Whip (or some object) is attached to the object
                G += np.dot( self.mjData.get_geom_jacp(  "geom_EE"    ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.Mw  * self.g  )

        elif self.n_limbs == 3:
            raise NotImplementedError( )


        return G


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


class JointImpedanceController( ImpedanceController ):

    """
        Description:
        ----------
            Class for a Joint Impedance Controller
            First order impedance controller with gravity compenation

    """

    def __init__( self, mjModel, mjData, mjArgs, is_grav_comps = True ):

        super().__init__( mjModel, mjData, mjArgs )

        if   self.n_act == 2:   # 2DOF Robot

            c = 0.1
            self.K = np.array( [ [ 29.50, 14.30 ] ,
                                 [ 14.30, 39.30 ] ] )
            self.B = c * self.K

        elif self.n_act == 4:   # 4DOF Robot

            # c = 0.05
            c = 0.1
            self.K = np.array( [ [ 17.40, 4.70, -1.90, 8.40 ] ,
                                 [  9.00, 33.0,  4.40, 0.00 ] ,
                                 [ -13.6, 3.00,  27.7, 0.00 ] ,
                                 [  8.40, 0.00,  0.00, 23.2 ] ] )
            self.B = c * self.K


        # [2DOF Robot] 5 movement parameters in total - Intial posture (2), Final posture (2) and duration (1)
        # [4DOF RObot] 9 movement parameters in total - Intial posture (4), Final posture (4) and duration (1)
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.3.] Implementation
        # [REF] [Moses C. Nah] Ibid.        ,                                                              [Section 8.2.2.] Zero-Torque Trajectory
        self.n_mov_pars     = 2 * self.n_act + 1                                    # 2 *, for initial/final postures and +1 for movement duration
        self.mov_parameters = None                                                  # The actual values of the movement parameters, initializing it with random values
        self.n_ctrl_pars    = [ self.n_mov_pars, self.n_act ** 2, self.n_act ** 2 ] # The number of ctrl parameters. This definition would be useful for the optimization process.
                                                                                    # K and B has n^2 elements, hence self.n_act ** 2

        self.is_grav_comps = is_grav_comps
        self.ctrl_par_names = [ "mov_parameters", "K", "B" ]                    # Useful for self.set_ctrl_par method

    def get_G( self ):

        if self.is_grav_comps:

            if   self.n_limbs == 2:

                # Torque for Gravity compensation is simply tau = J^TF
                # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
                G = np.dot( self.mjData.get_site_jacp( "upperArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[0] * self.g  )  \
                  + np.dot( self.mjData.get_site_jacp(  "foreArmCOM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[1] * self.g  )

                if "_w_" in self.mjArgs[ 'modelName' ]: # If a Whip (or some object) is attached to the object
                    G += np.dot( self.mjData.get_geom_jacp(  "geom_EE"    ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.Mw  * self.g  )

            elif self.n_limbs == 3:
                raise NotImplementedError( )

        else:
            G = np.zeros( self.n_act )

        return G


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
        self.ZFT_func_pos = min_jerk_traj( self.t_sym, pi, pf, D )
        self.ZFT_func_vel = [ sp.diff( tmp, self.t_sym ) for tmp in self.ZFT_func_pos ]

        # Lambdify the functions
        # [TIP] This is necessary for computation Speed!
        self.ZFT_func_pos = lambdify( self.t_sym, self.ZFT_func_pos )
        self.ZFT_func_vel = lambdify( self.t_sym, self.ZFT_func_vel )


    def get_ZFT( self, time ): # ZFT and ZTT will be the same for this code.

        D = self.mov_parameters[ -1 ]                                           # Last element is duration
        t = D if time >= D else time                                            # Rectifying the time value if time is larger than D
                                                                                # This means that the ZFT of the controller remains at final posture.
        x0  = np.array( self.ZFT_func_pos( t ) )
        dx0 = np.array( self.ZFT_func_vel( t ) )


        return x0, dx0



    def input_calc( self, start_time, current_time ):


        q  = self.mjData.qpos[ 0 : self.n_act ]                                 # Getting the relative angular position (q) and velocity (dq) of the shoulder and elbow joint, respectively.
        dq = self.mjData.qvel[ 0 : self.n_act ]

        if   current_time >= start_time:                                        # If time greater than startTime
            self.x0, self.dx0 = self.get_ZFT( current_time - start_time  )    # Calculating the corresponding ZFT of the given time. the startTime should be subtracted for setting the initial time as zero for the ZFT Calculation.
        else:
            self.x0, self.dx0 = q, dq                                         # Before start time, the posture should be remained at ZFT's initial posture

        tau_imp = np.dot( self.K, self.x0 - q ) + np.dot( self.B, self.dx0 - dq ) # Calculating the torque due to impedance
        tau_g   = self.get_G( )                                                 # Calculating the torque due to gravity compensation

        return self.mjData.ctrl, self.idx_act, tau_imp  + tau_g


if __name__ == "__main__":
    pass

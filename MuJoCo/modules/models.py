"""
    Contains the details of the upper-limb model parameters.
    Mostly the Coriolis, Mass matrix and other model parameters are parsed.

"""
# [Built-in modules]

# [3rd party modules]
import numpy as np
import sys
import time
import pickle

from   modules.utils        import my_print
from   modules.traj_funcs   import min_jerk_traj
from   modules.constants    import Constants

import matplotlib.pyplot as plt


class UpperLimbModel( ):

    def __init__( self, mjModel, mjData, mjArgs ):
        # Input model name and parsing the model values.

        # Parsing the model
        self.mjModel  = mjModel
        self.mjData   = mjData
        self.mjArgs   = mjArgs
        self.mjCtrl   = None

        self.n_limbs  = '-'.join( self.mjModel.body_names ).lower().count( 'arm' ) # The number of limbs of the controller. Checking bodies which contain "arm" as the name (Refer to xml model files)
        self.n_act    = len( self.mjModel.actuator_names )                      # The number of actuators, 2 for 2D model and 4 for 3D model

        if   self.n_limbs == 2:
            self.M  = [ self.get_property( 'body_upper_arm', 'mass'    ), self.get_property( 'body_fore_arm', 'mass'    ) ]
            self.I  = [ self.get_property( 'body_upper_arm', 'inertia' ), self.get_property( 'body_fore_arm', 'inertia' ) ]
            self.L  = [ self.length_elem2elem( 'geom_shoulder', 'geom_elbow'        ), self.length_elem2elem( 'geom_elbow' , 'geom_end_effector'  )  ]
            self.Lc = [ self.length_elem2elem( 'geom_shoulder', 'site_fore_arm_COM' ), self.length_elem2elem( 'geom_elbow' , 'site_upper_arm_COM' )  ]

        elif self.n_limbs == 3:
            NotImplementedError( )


    def length_elem2elem( self, elem_name1, elem_name2 ):
        type1 = self.get_elem_type( elem_name1 )
        type2 = self.get_elem_type( elem_name2 )

        # The euclidean distance between two elements, calling using "get_geom_xpos" or "get_site_xpos" or "get_body_xpos" methods
        return np.linalg.norm( getattr( self.mjData, "get_" + type1 + "_" + "xpos" )( elem_name1 )
                             - getattr( self.mjData, "get_" + type2 + "_" + "xpos" )( elem_name2 ) , ord = 1  )

    def get_elem_type( self, elem_name ):
        return elem_name.split( '_' )[ 0 ]                                      # Parse and get the first string before "_"

    def get_property( self, elem_name, prop_name ):
        # Get the property of the name

        # The name of the elements start with "XXXX_", hence getting the string before the underbar.
        type = self.get_elem_type( elem_name )

        for idx, s in enumerate( getattr( self.mjModel, type + "_" + "names" ) ):  # run through the list of "geom_names" or "body_names"
            if elem_name == s:
                tmp = getattr( self.mjModel, type + "_" + prop_name )
                return tmp[ idx ]

        # If couldn't match in list, raise error
        raise NameError( 'Cannot find geom_name with {0} in list, please check'.format( elem_name )  )

# 2DOF upper-limb model
class UpperLimbModelPlanar( UpperLimbModel ):

    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )

    def get_M( self, q ):

        # Get the q position, putting the whole
        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        M_mat = np.zeros( (2, 2) )

        M_mat[ 0, 0 ] = I1yy + I2yy + L1**2*M2 + Lc1**2*M1 + Lc2**2*M2 + 2*L1*Lc2*M2*np.cos(q[1])
        M_mat[ 0, 1 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
        M_mat[ 1, 0 ] = I2yy + Lc2*M2*(Lc2 + L1*np.cos(q[1]))
        M_mat[ 1, 1 ] = I2yy + Lc2**2*M2

        return M_mat

    def get_C( self, q, dq ):

        # Get the q position, putting the whole
        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        C_mat = np.zeros( (2, 2) )

        C_mat[ 0, 0 ] = -L1*Lc2*M2*np.sin(q[1])*dq[1]
        C_mat[ 0, 1 ] = -L1*Lc2*M2*np.sin(q[1])*(dq[0] + dq[1])
        C_mat[ 1, 0 ] = L1*Lc2*M2*np.sin(q[1])*dq[0]
        C_mat[ 1, 1 ] = 0

    def get_G( ):
        # Torque for Gravity compensation is simply tau = J^TF
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
        g     = mjModel.opt.gravity                                              # The gravity vector of the simulation
        G_mat = np.dot( self.mjData.get_site_jacp( "site_upper_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ 0 ] * g  )  \
              + np.dot( self.mjData.get_site_jacp(  "site_fore_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ 1 ] * g  )

        return G_mat


# 4DOF upper-limb model
class UpperLimbModelSpatial( UpperLimbModel ):

    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )

    def get_M( self, q ):

        # Get the q position, putting the whole
        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        M_mat = np.zeros( (4, 4) )

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

    def get_C( self, q, dq ):

        M1,   M2 = self.M
        Lc1, Lc2 = self.Lc
        L1,   L2 = self.L
        I1xx, I1yy, I1zz = self.I[ 0 ]
        I2xx, I2yy, I2zz = self.I[ 1 ]

        C_mat = np.zeros( (4, 4) )

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

    def get_G( self ):
        # Torque for Gravity compensation is simply tau = J^TF
        # [REF] [Moses C. Nah] [MIT Master's Thesis]: "Dynamic Primitives Facilitate Manipulating a Whip", [Section 7.2.1.] Impedance Controller
        g     = self.mjModel.opt.gravity                                              # The gravity vector of the simulation
        G_mat = np.dot( self.mjData.get_site_jacp( "site_upper_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ 0 ] * g  )  \
              + np.dot( self.mjData.get_site_jacp(  "site_fore_arm_COM" ).reshape( 3, -1 )[ :, 0 : self.n_act ].T, - self.M[ 1 ] * g  )

        return G_mat
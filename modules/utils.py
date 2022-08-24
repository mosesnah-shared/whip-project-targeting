import re
import sys
import math

import numpy as np

from scipy.spatial.transform import Rotation as R


from   modules.constants import Constants as C

# Define functions to be imported when used "import *"
# __all__  = [ "my_parser", "min_jerk_traj", "str2float", "get_model_prop", "get_data_prop", "get_length", "make_whip_downwards", "quaternion2euler", "print_vars" ] 

def quat2angx( q ):

    theta = 2 * np.arccos( q[ 0 ] )

    axis = q[ 1: ]
    axis = axis/ np.sqrt( np.sum( axis**2 ) )

    return theta, axis


def rot2quat( R: np.ndarray ):
    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
    # [REF] From Johannes

    assert len( R ) == 3 and len( R[ 0 ] ) == 3

    q = np.zeros( 4 )

    R00 = np.trace( R )
    tmp = np.array( [ R00, R[ 0,0 ], R[ 1,1 ], R[ 2,2 ] ] )
    k = np.argmax( tmp )

    q[ k ] = 0.5 * np.sqrt( 1 + 2 * tmp[ k ] - R00 )

    if k == 0:
        q[ 1 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )

    elif k == 1:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 2, 1 ] - R[ 1, 2 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )

    elif k == 2:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 0, 2 ] - R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 1, 0 ] + R[ 0, 1 ] )
        q[ 3 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    elif k == 3:
        q[ 0 ] = 0.25/q[ k ] * ( R[ 1, 0 ] - R[ 0, 1 ] )
        q[ 1 ] = 0.25/q[ k ] * ( R[ 0, 2 ] + R[ 2, 0 ] )
        q[ 2 ] = 0.25/q[ k ] * ( R[ 2, 1 ] + R[ 1, 2 ] )

    if q[ 0 ] < 0 : q = -q

    return q




def quat2rot( quat: np.ndarray ):

    # [REF] https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html

    assert len( quat ) == 4

    q0, q1, q2 ,q3  = quat[:]    

    R = np.zeros( ( 3, 3 ) )

    R[ 0, 0 ] = q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    R[ 0, 1 ] = 2 * q1 * q2 - 2 * q0 * q3
    R[ 0, 2 ] = 2 * q1 * q3 + 2 * q0 * q2

    R[ 1, 0 ] = 2 * q1 * q2 + 2 * q0 * q3
    R[ 1, 1 ] = q0 ** 2 - q1 ** 2 + q2 ** 2 - q3 ** 2
    R[ 1, 2 ] = 2 * q2 * q3 - 2 * q0 * q1

    R[ 2, 0 ] = 2 * q1 * q3 - 2 * q0 * q2
    R[ 2, 1 ] = 2 * q2 * q3 + 2 * q0 * q1
    R[ 2, 2 ] = q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2

    return R



def min_jerk_traj( t: float, ti: float, tf: float, pi: float, pf: float, D: float ):
    """
        Returning the 1D position and velocity data at time t of the minimum-jerk-trajectory ( current time )
        Time should start at t = 0
        Note that the minimum-jerk-trajectory remains at the initial (respectively, final) posture before (after) the movement.
        Arguments
        ---------
            [1] t : current time
            [2] ti: start of the movement
            [3] tf: end   of the movement
            [4] pi: initial ( reference ) posture
            [5] pf: final   ( reference ) posture
            [6]  D: duration
    """

    assert  t >=  0 and ti >= 0 and tf >= 0 and D >= 0
    assert tf >= ti

    if   t <= ti:
        pos = pi
        vel = 0

    elif ti < t <= tf:
        tau = ( t - ti ) / D                                                # Normalized time
        pos =    pi + ( pf - pi ) * ( 10 * tau ** 3 - 15 * tau ** 4 +  6 * tau ** 5 )
        vel = 1 / D * ( pf - pi ) * ( 30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4 )

    else:
        pos = pf
        vel = 0

    return pos, vel

def quat2euler( quat: np.ndarray ):                                         
    """
        Description
        -----------
        Converting a R4 quaternion vector (w, x, y, z) to Euler Angle (Roll, Pitch, Yaw)
        This code is directly from the following reference
        [REF] https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr

        Arguments
        ---------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) quatVec     List          The quaternion vector, ordered in w, x, y and z

        Outputs
        --------
            [NAME]                   [TYPE]        [DESCRIPTION]
            (1) yaw, pitch, roll                   The euler angles of the given quaternion vector.
    """

    assert len( quat ) == 4

    w, x, y ,z  = quat[:]

    t0     =       + 2.0 * ( w * x + y * z )
    t1     = + 1.0 - 2.0 * ( x * x + y * y )
    roll   = math.atan2( t0, t1 )

    t2     = + 2.0 * ( w * y - z * x )
    t2     = + 1.0 if t2 > +1.0 else t2
    t2     = - 1.0 if t2 < -1.0 else t2
    pitch  = math.asin( t2 )

    t3     =       + 2.0 * ( w * z + x * y )
    t4     = + 1.0 - 2.0 * ( y * y + z * z )
    yaw    = math.atan2( t3, t4 )

    return yaw, pitch, roll

def str2float( string2parse : str ):
    """
        Return A list of float that is parsed from given string s
    """

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", string2parse ) ]


def get_model_prop( mj_model, elem_name: str, name: str, prop_name:str ):
    """
        A method which simplifies the sentence for calling the property in interest
        If executes the following sentence.
            mj_model."elem_name" + "prop_name", 

            [Example] mj_data.body_mass

            name is needed for finding that value. 
    """

    # Saving the method (mth) that we will use. 
    mth = getattr( mj_model, "_".join( [ elem_name, "name2id" ] ) )

    # Returning the value.
    return getattr( mj_model, "_".join( [ elem_name, prop_name ] ) )[  mth( "_".join( [ elem_name, name ]  ) )  ]

def get_data_prop( mj_model, mj_data, elem_name: str, name: str, prop_name:str ):
    """
        A method which simplifies the sentence for calling the property in interest
        If executes the following sentence.
            mj_data."elem_name" + "prop_name", 

            [Example] mj_data.body_mass

            name is needed for finding that value. 
    """

    # Saving the method (mth) that we will use. 
    mth = getattr( mj_model, "_".join( [ elem_name, "name2id" ] ) )

    # Returning the value.
    return getattr( mj_data, "_".join( [ elem_name, prop_name ] ) )[  mth( "_".join( [ elem_name, name ]  ) )  ]


def get_length( mj_model, mj_data, elem1_type:str, elem1_name:str, elem2_type:str, elem2_name:str ):
    """
        Get the Euclidean distance between two elements. 

        Arguments
        --------
            [1] elem1_type: "site" or "body" or "geom" etc. 
            [2] elem1_name: name of element 1
            [3] elem2_type: "site" or "body" or "geom" etc. 
            [4] elem2_name: name of element 2

        This function will eventually derive the distance between 
        {elem1_type}_{elem1_name} and {elem2_type}_{elem2_name}.

        The crucial point is that we should use "xpos" rather than "pos", because the former one returns the Cartesian coord. 

        [Example]
            - length_elem2elem( mj_data, "site", "upper_arm_end", "site", "fore_arm_end" )
              returns the distance between "site_upper_arm_end" and "site_fore_arm_end".

    """

    return np.linalg.norm( get_data_prop( mj_model, mj_data, elem1_type, elem1_name, "xpos" ) - get_data_prop( mj_model, mj_data, elem2_type, elem2_name, "xpos" )  , ord = 2  )

def make_whip_downwards( sim ):
    """
        Get the qpos values for making the whip downwards
    """

    # The model should contain "body_whip_node1".
    # This body is the starting point of the whip
    # Note that we can use "site" instead, but since we need to convert from quaternion to Euler angles, 
    # site cannot be used since the "xquat" cannot be collected via "site". 
    assert "body_whip_node1" in sim.mj_model.body_names

    _, pitch, roll = quat2euler( sim.mj_data.get_body_xquat(  "body_whip_node1" ) )

    n_act = len( sim.mj_model.actuator_names )

    # For a 2DOF robot
    if   n_act == 2:
        sim.mj_data.qpos[ n_act ] = + pitch if round( roll ) == 0 else np.pi - pitch

    # For a 4DOF robot
    elif n_act == 4: 
        sim.mj_data.qpos[ n_act     ] = - roll  
        sim.mj_data.qpos[ n_act + 1 ] = + pitch 

    sim.mj_sim.forward( )

def print_vars( vars2print: dict , save_dir = sys.stdout ):
    """
        Print out all the details of the variables to the standard output + file to save. 
    """

    # Iterate Through the dictionary for printing out the values. 
    for var_name, var_vals in vars2print.items( ):

        # Check if var_vals is a list or numpy's ndarray else just change it as string 
        if   isinstance( var_vals, ( list, np.ndarray ) ):
            
            # First, change the list to numpy array to make the problem easier 
            var_vals = np.array( var_vals ) if isinstance( var_vals, list ) else var_vals

            # If the numpy array is
            var_vals = np.array2string( var_vals.flatten( ), separator =', ', floatmode = 'fixed' )

        else:
            var_vals = str( var_vals )

        print( f'[{var_name}]: {var_vals}', file = save_dir )

import re
import math
import numpy as np

# Define functions to be imported when used "import *"
__all__  = [ "str2float", "get_model_par", "get_length", "make_whip_downwards", "quaternion2euler" ] 


def quaternion2euler( quat: np.ndarray ):                                         
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

def get_model_par( mj_model, elem_name: str, name: str, prop_name:str ):
    """
        A method which simplifies the sentence for calling the values in interest
        If mj_model is the mujoco py's model, we retrive the value of

            mj_model."elem_name" + "prop_name", 

            [Example] mj_model.body_mass

            name is needed for finding that value. 
    """

    # Saving the method (mth) that we will use. 
    mth = getattr( mj_model, "_".join( [ elem_name, "name2id" ] ) )

    # Returning the value.
    return getattr( mj_model, "_".join( [ elem_name, prop_name ] ) )[  mth( "_".join( [ elem_name, name ]  ) )  ]


def get_length( mj_model, elem1_type:str, elem1_name:str, elem2_type:str, elem2_name:str ):
    """
        Get the Euclidean distance between two elements. 

        Arguments
        --------
            [1] elem1_type: "site" or "body" or "geom" etc. 
            [2] elem1_name: name of element 1
            [3] elem2_type: "site" or "body" or "geom" etc. 
            [4] elem2_name: name of element 2

        This function will eventually derive the distance between 
        {elem1_type}_{elem1_name} and {elem2_type}_{elem2_name}

        [Example]

        length_elem2elem( mj_data, "site", "upper_arm_end", "site", "fore_arm_end" )

        returns the distance between "site_upper_arm_end" and "site_fore_arm_end".

    """

    return np.linalg.norm( get_model_par( mj_model, elem1_type, elem1_name, "pos" ) - get_model_par( mj_model, elem2_type, elem2_name, "pos" )  , ord = 2  )

def make_whip_downwards( sim ):
    """
        Get the qpos values for making the whip downwards
    """

    # The model should contain "body_whip_node1".
    # This body is the starting point of the whip
    # Note that we can use "site" instead, but since we need to convert from quaternion to Euler angles, 
    # site cannot be used since the "xquat" cannot be collected via "site". 
    assert "body_whip_node1" in sim.mj_model.body_names

    _, pitch, roll = quaternion2euler( sim.mj_data.get_body_xquat(  "body_whip_node1" ) )

    n_act = len( sim.mj_model.actuator_names )

    # For a 2DOF robot
    if   n_act == 2:
        sim.mj_data.qpos[ n_act ] = + pitch if round( roll ) == 0 else np.pi - pitch

    # For a 4DOF robot
    elif n_act == 4: 
        sim.mj_data.qpos[ n_act     ] = - roll  
        sim.mj_data.qpos[ n_act + 1 ] = + pitch 

    sim.mj_sim.forward( )
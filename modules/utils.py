import os
import re
import sys
import shutil
import time, datetime
import math  as myMath
import glob

import cv2
import numpy                 as np
import xml.etree.ElementTree as ET

import sympy as sp

from scipy.special    import lambertw
from scipy.integrate  import quad
from scipy.spatial.transform import Rotation as R

# [Local modules]
from modules.constants import Constants as C

def length_elem2elem( mjModel, mjData, elem_name1, elem_name2 ):
    # The euclidean distance between two elements, calling using "get_geom_xpos" or "get_site_xpos" or "get_body_xpos" methods
    return np.linalg.norm( getattr( mjData, "get_" + type1 + "_" + "xpos" )( elem_name1 )
                         - getattr( mjData, "get_" + type2 + "_" + "xpos" )( elem_name2 ) , ord = 2  )

def snake2camel( s: str ):
    return ''.join( word.title() for word in s.split( '_' ) )

def camel2snake( s: str ):
    return re.sub( r'(?<!^)(?=[A-Z])', '_', s ).lower()

def clear_dir( dir ):
    """ Cleaning up the contents in the directory """

def rot2quat( rot ):
    # Taking the SO(3) matrix as an input and return the quaternion
    # return quat
    pass

def euler2quaternion( euler_angs ):
    """
        Description
        -----------
            This code is directly from the following reference
            [REF] https://computergraphics.stackexchange.com/questions/8195/how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
            Converting a R4 quaternion vector (w, x, y, z) to Euler Angle (Roll, Pitch, Yaw)

        Arguments
        ---------
            [NAME]                   [TYPE]        [DESCRIPTION]
            (1) yaw, pitch, roll                   The euler angles of the given quaternion vector.

        [OUTPUTS]
        -----------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) quatVec     List          The quaternion vector, ordered in w, x, y and z


    """
    yaw, pitch, roll  = euler_angs[ : ]

    cy = np.cos( yaw   * 0.5 )
    sy = np.sin( yaw   * 0.5 )
    cp = np.cos( pitch * 0.5 )
    sp = np.sin( pitch * 0.5 )
    cr = np.cos( roll  * 0.5 )
    sr = np.sin( roll  * 0.5 )


    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;

    return w,x,y,z


def quaternion2euler( quatVec ):                                                # Inputting quaternion matrix and outputing the yaw, pitch, roll of the euler angle.
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

    if len( quatVec ) != 4:
        raise ValueError( "Wrong size of input argument. Given size is [{0:d}] while it should be 4".format(
                                                                    len( quatVec ) ) )

    w, x, y ,z  = quatVec[:]

    t0     =       + 2.0 * ( w * x + y * z )
    t1     = + 1.0 - 2.0 * ( x * x + y * y )
    roll   = myMath.atan2( t0, t1 )

    t2     = + 2.0 * ( w * y - z * x )
    t2     = + 1.0 if t2 > +1.0 else t2
    t2     = - 1.0 if t2 < -1.0 else t2
    pitch  = myMath.asin( t2 )

    t3     =       + 2.0 * ( w * z + x * y )
    t4     = + 1.0 - 2.0 * ( y * y + z * z )
    yaw    = myMath.atan2( t3, t4 )

    return yaw, pitch, roll

def str2float( s : str ):
    """

        Args:
        ----------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) s           str           The string which will be parsed to float array

        Returns:
        ----------
            A list of float that is parsed from given string s

    """

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", s ) ]


def solve_eq_posture( q0 ):

    q1_0 = q0[ 0 ]
    q2_0 = q0[ 1 ]
    q3_0 = q0[ 2 ]
    q4_0 = q0[ 3 ]

    q1 = sp.Symbol( 'q1' )
    q2 = sp.Symbol( 'q2' )
    q3 = sp.Symbol( 'q3' )
    q4 = sp.Symbol( 'q4' )

    eqn1 = 0.52444712807465876380774716380984*sp.cos(q2)*sp.sin(q1) - 0.12721953522735995889547666592989*sp.cos(q1)*sp.sin(q2) - 0.05501625493258266441642945210333*sp.sin(q4)*(sp.sin(q1)*sp.sin(q3) + sp.cos(q1)*sp.cos(q3)*sp.sin(q2)) - 0.063807174539763700238381716189906*sp.cos(q1)*sp.cos(q2)*sp.sin(q4) - 0.042749427781976545581699156173272*sp.cos(q1)*sp.cos(q4)*sp.sin(q2) + 0.1762293392050615636890142923221*sp.cos(q2)*sp.cos(q4)*sp.sin(q1) + 0.1762293392050615636890142923221*sp.cos(q1)*sp.cos(q3)*sp.sin(q4) - 0.063807174539763700238381716189906*sp.cos(q3)*sp.cos(q4)*sp.sin(q1) + 0.042749427781976545581699156173272*sp.cos(q1)*sp.cos(q2)*sp.sin(q3)*sp.sin(q4) + 0.063807174539763700238381716189906*sp.cos(q1)*sp.cos(q4)*sp.sin(q2)*sp.sin(q3) + 0.1762293392050615636890142923221*sp.sin(q1)*sp.sin(q2)*sp.sin(q3)*sp.sin(q4) + q1 - q1_0
    eqn2 = 0.1966778910733553153988850681344*sp.cos(q1)*sp.sin(q2) - 0.12721953522735995889547666592989*sp.cos(q2)*sp.sin(q1) + 0.020788410744410568131712579997838*sp.sin(q4)*(sp.sin(q1)*sp.sin(q3) + sp.cos(q1)*sp.cos(q3)*sp.sin(q2)) + 0.015478241093474287559672575298464*sp.cos(q1)*sp.cos(q2)*sp.sin(q4) + 0.066089435759419945526360606891103*sp.cos(q1)*sp.cos(q4)*sp.sin(q2) - 0.042749427781976545581699156173272*sp.cos(q2)*sp.cos(q4)*sp.sin(q1) - 0.042749427781976545581699156173272*sp.cos(q1)*sp.cos(q3)*sp.sin(q4) + 0.015478241093474287559672575298464*sp.cos(q3)*sp.cos(q4)*sp.sin(q1) - 0.066089435759419945526360606891103*sp.cos(q1)*sp.cos(q2)*sp.sin(q3)*sp.sin(q4) - 0.015478241093474287559672575298464*sp.cos(q1)*sp.cos(q4)*sp.sin(q2)*sp.sin(q3) - 0.042749427781976545581699156173272*sp.sin(q1)*sp.sin(q2)*sp.sin(q3)*sp.sin(q4) + q2 - q2_0
    eqn3 = 0.1637248203220158515591720060911*sp.cos(q2)*sp.sin(q1) - 0.061864967327922570916598488111049*sp.cos(q1)*sp.sin(q2) - 0.083555731966853175052278857037891*sp.sin(q4)*(sp.sin(q1)*sp.sin(q3) + sp.cos(q1)*sp.cos(q3)*sp.sin(q2)) - 0.019919678510073035582195188908372*sp.cos(q1)*sp.cos(q2)*sp.sin(q4) - 0.020788410744410568131712579997838*sp.cos(q1)*sp.cos(q4)*sp.sin(q2) + 0.05501625493258266441642945210333*sp.cos(q2)*sp.cos(q4)*sp.sin(q1) + 0.05501625493258266441642945210333*sp.cos(q1)*sp.cos(q3)*sp.sin(q4) - 0.019919678510073035582195188908372*sp.cos(q3)*sp.cos(q4)*sp.sin(q1) + 0.020788410744410568131712579997838*sp.cos(q1)*sp.cos(q2)*sp.sin(q3)*sp.sin(q4) + 0.019919678510073035582195188908372*sp.cos(q1)*sp.cos(q4)*sp.sin(q2)*sp.sin(q3) + 0.05501625493258266441642945210333*sp.sin(q1)*sp.sin(q2)*sp.sin(q3)*sp.sin(q4) + q3 - q3_0
    eqn4 = 0.046062245513354471704303705337225*sp.cos(q1)*sp.sin(q2) - 0.18988602913048024944941971625667*sp.cos(q2)*sp.sin(q1) + 0.019919678510073035582195188908372*sp.sin(q4)*(sp.sin(q1)*sp.sin(q3) + sp.cos(q1)*sp.cos(q3)*sp.sin(q2)) + 0.10117159250577656415259752975544*sp.cos(q1)*sp.cos(q2)*sp.sin(q4) + 0.015478241093474287559672575298464*sp.cos(q1)*sp.cos(q4)*sp.sin(q2) - 0.063807174539763700238381716189906*sp.cos(q2)*sp.cos(q4)*sp.sin(q1) - 0.063807174539763700238381716189906*sp.cos(q1)*sp.cos(q3)*sp.sin(q4) + 0.10117159250577656415259752975544*sp.cos(q3)*sp.cos(q4)*sp.sin(q1) - 0.015478241093474287559672575298464*sp.cos(q1)*sp.cos(q2)*sp.sin(q3)*sp.sin(q4) - 0.10117159250577656415259752975544*sp.cos(q1)*sp.cos(q4)*sp.sin(q2)*sp.sin(q3) - 0.063807174539763700238381716189906*sp.sin(q1)*sp.sin(q2)*sp.sin(q3)*sp.sin(q4) + q4 - q4_0

    sol = sp.solvers.nsolve( ( eqn1, eqn2, eqn3, eqn4 ), ( q1, q2, q3, q4 ), q0  )
    sol = np.array( sol )
    return np.array( [ sol[ 0 ][ 0 ], sol[ 1 ][ 0 ], sol[ 2 ][ 0 ], sol[ 3 ][ 0 ] ] )

if __name__ == '__main__':
    pass

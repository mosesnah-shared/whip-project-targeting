# [Built-in modules]
import os
import re
import sys
import shutil
import time, datetime
import math  as myMath
import glob

# [3rd party modules]
import cv2
import numpy                 as np
import xml.etree.ElementTree as ET

import sympy as sp
from sympy.utilities.lambdify import lambdify, implemented_function

from scipy.special    import lambertw
from scipy.integrate  import quad

# [Local modules]
from modules.constants import Constants

class MyVideo:
    """
        Description
        ----------

        Arguments
        ---------

        Returns
        -------
    """
    def __init__( self, vid_dir = None, height = 1440, width = 850, fps = 60 ):
        self.height    = height
        self.width     = width
        self.vid_dir   = vid_dir if not None else "."
        self.fps       = fps

        fourcc         = cv2.VideoWriter_fourcc( *'MP4V' )                      # 4-character code of codec used to compress the frames.
                                                                                # For example, VideoWriter::fourcc('P','I','M','1') is a MPEG-1 codec,
                                                                                #              VideoWriter::fourcc('M','J','P','G') is a motion-jpeg codec etc.
                                                                                # List of codes can be obtained at Video Codecs by FOURCC page.
        # self.outVideo  = cv2.VideoWriter( self.vid_dir + "/video.mp4", fourcc, fps, ( self.height, self.width ) )
        self.outVideo  = cv2.VideoWriter( self.vid_dir + "/video.mp4", fourcc, fps, ( self.height, self.width ) )

    def write( self, myViewer ):
        data = myViewer.read_pixels( self.height, self.width, depth = False )   # Get the pixel from the render screen
        data = cv2.cvtColor( data, cv2.COLOR_BGR2RGB )

        # data = cv2.resize( data,( self.height, self.width  ) )

        self.outVideo.write( np.flip( data, axis = 0 ) )

    def release( self ):
        self.outVideo.release()

def length_elem2elem( mjModel, mjData, elem_name1, elem_name2 ):
    type1 = get_elem_type( mjModel, elem_name1 )
    type2 = get_elem_type( mjModel, elem_name2 )

    # The euclidean distance between two elements, calling using "get_geom_xpos" or "get_site_xpos" or "get_body_xpos" methods
    return np.linalg.norm( getattr( mjData, "get_" + type1 + "_" + "xpos" )( elem_name1 )
                         - getattr( mjData, "get_" + type2 + "_" + "xpos" )( elem_name2 ) , ord = 2  )


def get_elem_type( mjModel, elem_name ):
    """
        The naming convention of our mujoco simulation is "{elem}_name", where elem = [geom, site, body]
        The string before the first underbar '_' describes the elem(ent) of the model.
        This function parses the string and returns the first string (i.e., the element of the model)
    """
    return elem_name.split( '_' )[ 0 ]                                      # Parse and get the first string before "_"

def get_property( mjModel, elem_name, prop_name ):
    # Get the property of the name

    # The name of the elements start with "XXXX_", hence getting the string before the underbar.
    type = get_elem_type( mjModel, elem_name )

    for idx, s in enumerate( getattr( mjModel, type + "_" + "names" ) ):  # run through the list of "geom_names" or "body_names"
        if elem_name == s:
            tmp = getattr( mjModel, type + "_" + prop_name )
            return tmp[ idx ]

    # If couldn't match in list, raise error
    raise NameError( 'Cannot find geom_name with {0} in list, please check'.format( elem_name )  )


def snake2camel( s ):
    """
        Switch string s from snake_form_naming to CamelCase
    """

    return ''.join( word.title() for word in s.split( '_' ) )

def camel2snake( s ):
    """
        Switch string s from CamelCase to snake_form_naming
        [REF] https://stackoverflow.com/questions/1175208/elegant-python-function-to-convert-camelcase-to-snake-case
    """
    re.sub( r'(?<!^)(?=[A-Z])', '_', s ).lower()

def clear_dir( dir ):
    """ Cleaning up the contents in the directory """



def args_cleanup( args, s ):
    """
        Description
        -----------
            Clean-up the substring s for keys in args

        Arguments
        ---------
            args: The dictionary to be parsed
            s   : Substring to be discarded. e.g. s = '--', then "--record" --> "record"

    """
    if not isinstance( args, dict ) or not isinstance( s, str ):
        raise ValueError( "Wrong input type. args should be type dict and s should be type str. {0:} and {1:} are rather given".format(
                                                                                            type( args ), type( str ) ) )

    for old_key in list( args ) :
        new_key = old_key.replace( s, '' )
        args[ new_key ] = args.pop( old_key )

    return args

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

def str2bool( s ):
    """

        Description:
        ----------
        Converting an input string to a boolean

        Arguments:
        ----------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) s           dict, str     The string which

        Returns:
        ----------
            True/False depending on the given input strin gv

    """
    if isinstance( s, dict ):
        for key, _ in s.items():
            s[ key ] = str2bool( s[ key ] )
    else:
        return v.lower() in ( "yes", "true", "t", "1" )

def str2float( s ):
    """

        Description:
        ----------
        Converting an input string to a float arraay

        Arguments:
        ----------
            [NAME]          [TYPE]        [DESCRIPTION]
            (1) s           str           The string which will be parsed to float array

        Returns:
        ----------
            The parsed float array

    """
    if not isinstance( s, str ):
        raise ValueError( "Input argument should be string, but {} is given".format( type( s ) ) )

    return [ float( i ) for i in re.findall( r"[-+]?\d*\.\d+|[-+]?\d+", s ) ]

def my_mkdir(  ):

    dir = Constants.TMP_DIR                                                     # Temporarily saving at tmp
    dir += datetime.datetime.now().strftime( "%Y%m%d_%H%M%S/" )                 # Appending the date when this directory is called.
    if not os.path.exists( dir ):                                               # If directory not exist
        os.makedirs( dir, exist_ok = True )                                     # mkdir -p functionality via exist_ok

    return dir


def my_mvdir( from_dir, to_dir ):
    shutil.move( from_dir , to_dir )


def my_rmdir( dir ):

    if not isinstance( dir, str ):
        raise ValueError( "Input directory should be a str, {} is given".format( type ( dir ) ) )

    try:
        shutil.rmtree( dir  )
    except:
        print( "{0:s} Doesn't exist, hence cannot remove the directory".format( dir ) )

    print( "Erasing Directory [{0:s}]".format( dir ) )

def my_print( **kwargs ):
    """
        Description:
        ----------
            ** double asterisk means giving the argument as dictionary
            By using double asterisk "kwargs" as input argument,

        Arguments:
        ----------

        Returns:
        ----------
    """

    prec = kwargs[ "prec" ] if "prec" in kwargs else 5
    f    = kwargs[ "file" ] if "file" in kwargs else sys.stdout                 # If there is a keyword called "file" then use that as our standard output

    tmpMaxLen = len( max( kwargs.keys( ), key = len ) )                         # Getting the maximum length of a string list

    for args in kwargs:

        if 'file' == args.lower( ):
            # Ignore the file's value, since it should not be added to the "output.txt" log file.
            continue


        print( "[{1:{0}s}]:".format( tmpMaxLen, args ), end = ' ', file = f )   # Printing out the name of the array
                                                                                # {1:{0}s} Enables to set a variable as format length.
        tmpData = kwargs[ args ]

        if   isinstance( tmpData, ( float, int ) ):
            tmpPrint = "{2:{1}.{0}f}".format( prec, prec + 2, tmpData )

        elif isinstance( tmpData, list  ):
            tmpPrint = np.array2string( np.array( tmpData ).flatten(), precision = prec, separator = ',' )

        elif isinstance( tmpData, np.ndarray  ):
            tmpPrint = np.array2string( tmpData.flatten()            , precision = prec, separator = ',' )

        elif isinstance( tmpData, str   ):
            tmpPrint = tmpData

        elif tmpData is None:
            tmpPrint = "None"

        else:
            raise ValueError( "CHECK INPUT")

        print( tmpPrint, file = f )

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

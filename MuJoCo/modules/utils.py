# [Built-in modules]
import os
import re
import sys
import shutil
import time, datetime
import math  as myMath

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
    def __init__( self, vid_dir = None, height = 2880, width = 1700, fps = 60 ):
        self.height    = height
        self.width     = width
        self.vid_dir   = vid_dir if not None else "."
        self.fps       = fps

        fourcc         = cv2.VideoWriter_fourcc( *'MP4V' )                      # 4-character code of codec used to compress the frames.
                                                                                # For example, VideoWriter::fourcc('P','I','M','1') is a MPEG-1 codec,
                                                                                #              VideoWriter::fourcc('M','J','P','G') is a motion-jpeg codec etc.
                                                                                # List of codes can be obtained at Video Codecs by FOURCC page.
        # self.outVideo  = cv2.VideoWriter( self.vid_dir + "/video.mp4", fourcc, fps, ( self.height, self.width ) )
        self.outVideo  = cv2.VideoWriter( self.vid_dir + "/video.mp4", fourcc, fps, ( self.height//2, self.width//2 ) )

    def write( self, myViewer ):
        data = myViewer.read_pixels( self.height, self.width, depth = False )   # Get the pixel from the render screen
        data = cv2.resize( data,( self.height//2, self.width//2  ) )

        self.outVideo.write( np.flip( data, axis = 0 ) )

    def release( self ):
        self.outVideo.release()

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

def my_mkdir( dir = './results/' ):

    if dir is not None and dir[ -1 ] != "/":                                    # Quick Check of whether result_dir has backslash "/" at the end
        dir += "/"                                                              # Append the backslash

    dir  += datetime.datetime.now().strftime( "%Y%m%d_%H%M%S" )                 # Appending the date when this directory is called.
    if not os.path.exists( dir ):                                               # If directory not exist
        os.makedirs( dir )                                                      # Make the directory

    return dir + "/"

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


if __name__ == '__main__':
    pass

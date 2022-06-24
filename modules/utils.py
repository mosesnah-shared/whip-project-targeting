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
    return re.sub( r'(?<!^)(?=[A-Z])', '_', s ).lower( )


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

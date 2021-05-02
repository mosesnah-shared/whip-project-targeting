# [Built-in modules]
import os
import re
import sys
import shutil
import time, datetime
import math  as myMath

# [3rd party modules]
import numpy as np
import sympy as sp
from sympy.utilities.lambdify import lambdify, implemented_function
from modules.constants import Constants

"""
    This python script defines the basis functions/miscellaneous functions that will be used for multiple situations

"""

# Define mother class for all Trajectory
class Trajectory( ):
    """
        Description:
        -----------
            Parent class for the controllers
    """


    def __init__( self, t ):
        """

        """
        self.t_sym       = t    # The symbolic form of time

        # The symbolic form of the trajectory
        self.traj        = None
        self.dtraj       = None
        self.ddtraj      = None

        # The lambdified functions, this shrinks the computation time
        self.traj_func   = None
        self.dtraj_func  = None
        self.ddtraj_func = None

        # Type of the trajectory, usually the name of it.
        self.traj_type   = None


class MinimumJerkTrajectory( Trajectory ):
    """
        Description:
        ----------
            The lambda function for the minimum-jerk-trajectory, suggested by Prof. Hogan.
            It is a 5-th order polynomial which minimizes the mean-squared-jerk value.
            [REF] Flash, Tamar, and Neville Hogan. "The coordination of arm movements: an experimentally confirmed mathematical model." Journal of neuroscience 5.7 (1985): 1688-1703.

        Arguments:
        ----------
            [1]  t: (sym)   The time symbol
            [2] pi: (array) Initial posture
            [3] pf: (array)   Final posture
            [4]  D: (float) Duration it took from start to end

        Returns:
        ----------
            Function w.r.t. t function of the min_jerk_trajectory
    """

    def __init__( self, t, pi = None, pf = None, D = None):

        super().__init__( t )      # First initializing the trajectory

        self.traj_type = "minimum-jerk-trajectory"

        self.traj   = pi + ( pf - pi ) * ( 10 * np.power( t ,3 ) / ( D ** 3 ) - 15 * np.power( t , 4 ) / ( D ** 4 ) +  6 * np.power( t, 5 ) / ( D ** 5 ) )
        self.dtraj  = sp.diff( self.traj , self.t_sym )
        self.ddtraj = sp.diff( self.dtraj, self.t_sym )

        self.traj_func   = lambdify( self.t_sym, self.traj   )
        self.dtraj_func  = lambdify( self.t_sym, self.dtraj  )
        self.ddtraj_func = lambdify( self.t_sym, self.ddtraj )


if __name__ == "__main__":
    pass

# [Built-in modules]

# [3rd party modules]
import numpy as np
import sympy as sp
from modules.constants import Constants

try:
    import sympy as sp
    from sympy.utilities.lambdify import lambdify, implemented_function

except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install sympy, \
                                             Simply type pip3 install sympy. \
                                             Sympy is necessary for building ZFT Calculation)".format( e ) )

"""
    This python script defines the basis functions/miscellaneous functions that will be used for multiple situations

"""

# [TODO] [2021.07.20]
# Define mother class of Trajectory, and subsequently define others?
# Note that this implementation might be an overkill, i.e., the implementation might not be that meaningful for the code quality.
class Trajectory( ):
    """
        Description:
        -----------
            Parent class of trajectories.
            A lot of form of trajectories will be used throughout this code, hence defining a parent template for multiple child trajectories

    """

    def __init__( self ,traj_pars ):
        """
        Arguments:
        ----------
            [1]     t_sym: (sym)        The time symbol
            [2] traj_pars: (dictionary) Initial posture
        """

        self.t_sym       = sp.symbols( 't' )
        self.pars        = traj_pars
        self.n_pars      = None
        self.name        = None
        self.pars_names  = None                                            # The name of the trajectory parameters.
                                                                                # [Example] For min-jerk-traj, pi, pf, D

        # Usually, we are interested up to the 2nd derivative of trajectory.
        # Saving the lambdified function of the trajectory's position/velocity/acceleration.
        self.func_pos = None
        self.func_vel = None
        self.func_acc = None

    def check_pars( self, pars ):
        """
            Internal function which checks whether the input is properly given to the function
        """
        for key in pars.keys( ):
            if key not in self.pars_names:
                raise ValueError( "{0:s} is given as parameter, which is not defined in this class, Options: {1:s}".format( key, ", ".join( self.pars_names ) ) )

    def set_traj( self, traj_pars ):
        raise NotImplementedError( )


class MinJerkTrajectory( Trajectory ):
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
    def __init__( self, traj_pars ):
        super().__init__( traj_pars )

        self.name       = "Minimum Jerk Trajectory"
        self.n_pars     = sum( lst.size for lst in traj_pars.values() )
        self.pars_names = [ "pi", "pf", "D" ]

        self.check_pars( traj_pars )
        self.set_traj(   traj_pars )

    def set_traj( self, traj_pars ):

        # Quick check whether the input was given correctly.
        self.check_pars( traj_pars )

        pi = traj_pars[ "pi" ]
        pf = traj_pars[ "pf" ]
        D  = traj_pars[ "D"  ]
        t  = self.t_sym

        self.func_pos = pi + ( pf - pi ) * ( 10 * np.power( t ,3 ) / ( D ** 3 ) - 15 * np.power( t , 4 ) / ( D ** 4 ) +  6 * np.power( t, 5 ) / ( D ** 5 ) )
        self.func_vel = [ sp.diff( tmp, t ) for tmp in self.func_pos ]
        self.func_acc = [ sp.diff( tmp, t ) for tmp in self.func_vel ]

        # Lambdify the functions
        # [TIP] This is necessary for computation Speed!
        self.func_pos = lambdify( t, self.func_pos )
        self.func_vel = lambdify( t, self.func_vel )
        self.func_acc = lambdify( t, self.func_acc )



if __name__ == "__main__":
    pass

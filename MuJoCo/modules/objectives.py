import re
import numpy as np
import pprint

from modules.utils import length_elem2elem

try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

class Objective( ):
    """ Mother class of all objective functions """
    def __init__( self, mjModel, mjData, mjArgs ):
        self.mjModel     = mjModel
        self.mjData      = mjData
        self.mjArgs      = mjArgs

    # [TODO] [Moses C. Nah]
    # Making this to do things as 0.3 + 0.5 obj1
    # The reason why we've used class is because of static variables.
    # def __add__( self, other ):
    #     """ Modifying the __add__ function will help us use syntax like func1 + func2 """
    #     return self.output_calc( ) + other.output_calc( )
    #
    # def __mul__( self, other ):
    #     """ Modifying the __mul__ function will help us use syntax like 0.3 * func1 """
    #     return self.output_calc( ) * other if isinstance( other, int ) or isinstance( other, float ) else self.output_calc( ) * other.output_calc( )

    def __str__( self ):
        """ Starting and ending with __ are called "magic methods" [REF] https://www.tutorialsteacher.com/python/magic-methods-in-python """
        return str( vars( self ) )

    def output_calc( self ):
        NotImplementedError( )  # Imposing the child class to implement this function

class TargetState( Objective ):
    """ Trying to reach some target State  """

    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )

        self.targets = []

    def set_target( self, boundary ):
        # There will be a variable in focus, we need to "probe" that variable.
        # The boundary dictionary consists as follows:
        # [1] Which variable, is it the qpos or the qvel?
        # [2] For the variable defined in [1], which specific index?
        # [3] target value for that value
        # [4] And tolerance for that, +- [3]'s value
        self.targets.append( boundary )

    def output_calc( self ):

        # Iterating through the targets
        val = 0.0
        for target in self.targets:
            tmp = getattr( self.mjData, target[ "which_var"  ] )                # which_var is either "qpos" or "qvel"
            tmp = tmp[ target[ "idx" ]  ]                                       # Getting the value with corresponding index, idx is either list or scalar

            tmp_v = np.sqrt( np.sum( np.square( tmp - target[ "target_val" ] ) ) )
            if tmp_v <= target[ "tolerance" ]:
                tmp_v = 0.0

            val += tmp_v

        return val

class DistFromTip2Target( Objective ):
    """
        Getting the distance between the tip of the whip and target.

        In the .xml model file, The geometry name of target is named as "target",
        and the geometry name of the tip of the whip contains "tip" within the name.

        Retriving the distance between the tip of the whip and target.

    """
    def __init__( self, mjModel, mjData, mjArgs, tol = 1 ):
        """
            Arguments
            ---------
            tol: (int) The number of geom that we are accounting for the targeting task.
                       if tol = 5, then we are considering from geom21~25 for the distance.

        """

        super().__init__( mjModel, mjData, mjArgs )
        self.tol         = tol

        self.tip_name    = self._find_tip(  )                                   # Number of sub-models of the whip
        self.target_name = "geom_target"
        self.target_idx  = self.mjModel.geom_name2id( self.target_name )
        self.target_size = max( self.mjModel.geom_size[ self.target_idx ] )     # Get the size number of the target
        self.N           = int( re.search( r'\d+', self.tip_name ).group() )

        # The list of geometry that we are extracting for the targeting task
        if tol == 1:
            self.geom_list = list( )
        else:
            self.geom_list   = [ "geom_" + str( self.N - self.tol + i + 1 )  for i in range( self.tol - 1 )  ]

        self.geom_list.append( self.tip_name )


    def output_calc( self ):
        """ Calculate the objective function, which is the minimum distance between parts of the whip (ref. tol variable) and target. """
        lens = [ length_elem2elem( self.mjModel, self.mjData, geom, self.target_name ) for geom in self.geom_list  ]
        output = min( lens )

        # if target is hit by the whip, set output as output and change to green light!
        if output <= self.target_size + 0.012:
            output = 0.0  # For tolerance.
            self.mjModel.geom_rgba[ self.target_idx ] = [ 0, 1, 0, 1 ]

        return output

    def _find_tip( self ):

        for name in self.mjModel.geom_names:
            if "tip" in name:
                return name

        raise ValueError( "No word with tip on the model, to use class DistFromTip2Target we need to have a geom with name tip"           )


if __name__ == "__main__":
    pass

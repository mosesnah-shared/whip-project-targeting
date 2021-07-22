import re
import numpy as np
from modules.utils import length_elem2elem

try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

class DistFromTip2Target( ):
    """
        Getting the distance between the tip of the whip and target.

        In the .xml model file, The geometry name of target is named as "target",
        and the geometry name of the tip of the whip contains "tip" within the name.

        Retriving the distance between the tip of the whip and target.

    """
    def __init__( self, mjModel, mjData, tol = 5 ):
        """
            Arguments
            ---------
            tol: (int) The number of geom that we are accounting for the targeting task.
                       if tol = 5, then we are considering from geom21~25 for the distance.

        """

        self.mjModel     = mjModel
        self.mjData      = mjData
        self.tol         = tol

        self.tip_name    = self.find_tip(  )                                    # Number of sub-models of the whip
        self.target_name = "geom_target"
        self.target_idx  = self.mjModel.geom_name2id( self.target_name )
        self.target_size = max( self.mjModel.geom_size[ self.target_idx ] )     # Get the size number of the target

        self.N           = int( re.search( r'\d+', self.tip_name ).group() )

        # The list of geometry that we are extracting for the targeting task
        self.geom_list   = [ "geom_" + str( self.N - self.tol + i + 1 )  for i in range( self.tol - 1 )  ]
        self.geom_list.append( self.tip_name )


    def find_tip( self ):

        for name in self.mjModel.geom_names:
            if "tip" in name:
                return name

        raise ValueError( "No word with tip on the model, to use class DistFromTip2Target we need to have a geom with name tip"           )

    def output_calc( self ):
        """
            Calculate the objective function, which is the minimum distance between parts of the whip (ref. tol variable) and target.

        """
        lens = [ length_elem2elem( self.mjModel, self.mjData, geom, self.target_name ) for geom in self.geom_list  ]

        output = min( lens )

        # if target is hit by the whip, set output as output and change to green light!
        if output <= self.target_size:
            output = 0.0  # For tolerance.
            self.mjModel.geom_rgba[ self.target_idx ] = [0, 1, 0, 1]

        return output


if __name__ == "__main__":
    pass

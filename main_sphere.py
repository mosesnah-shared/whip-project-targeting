import os
import sys
import argparse
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import SphereController
from constants    import my_parser
from constants    import Constants  as C
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined in utils.py
parser = my_parser( )
args, unknown = parser.parse_known_args( )


if __name__ == "__main__":

    args.model_name = "simple_sphere"

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    ctrl = SphereController( my_sim, args, "example_sphere" )

    # Define the parameters of the sphere controller.
    my_sim.init( qpos = [ .0, .3, 0 ], qvel = np.zeros( 3 ))

    ctrl.set_desired_orientation( np.eye( 3 ) )
    my_sim.add_ctrl( ctrl )

    my_sim.run( )


    my_sim.close( )

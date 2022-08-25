import os
import sys
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation 
from controllers  import Excitator, Excitator_MJT
from constants    import my_parser
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined in utils.py
parser = my_parser( )
args, unknown = parser.parse_known_args( )


if __name__ == "__main__":

    args.model_name = "25DOF_whip"

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    # For simple sphere controller
    # ctrl = SphereController( my_sim, args, "example_sphere" )

    # For advanced sphere controller    
    ctrl = Excitator( my_sim, args, "excite_sine" )
    # ctrl = Excitator_MJT( my_sim, args, "excite_MJT" )

    # Define the parameters of the sphere controller.
    my_sim.init( qpos = [ 0. ], qvel = [ 0. ])

    ctrl.set_mov_pars( dpos = 0.3, D = 0.3, ti = args.start_time )
    my_sim.add_ctrl( ctrl )

    my_sim.run( )

    my_sim.close( )

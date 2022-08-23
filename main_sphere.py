import os
import sys
import argparse
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import SphereController
from constants    import Constants  as C
from MLmodule     import *
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined in utils.py
parser = my_parser( )
args, unknown = parser.parse_known_args( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    ctrl = SphereController( my_sim, args )
    my_sim.run( )

    print( f"The minimum value of this trial is { min( my_sim.obj_arr ):.5f}" )

    if args.is_save_data: 
        ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

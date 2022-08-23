"""

# ============================================================================= #
| Project:        [M3X Whip Project]
| Title:          mujoco-py scripts for running a whip-targeting simuation
| Author:         Moses C. Nah
| Email:          [Moses] mosesnah@mit.edu
| Creation Date:  Dec 8th,  2020
| Final Update:   Jun 20th, 2022
# ============================================================================= #

"""


import os
import sys
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
from objectives   import DistFromTip2Target
from constants    import my_parser
from constants    import Constants  as C
from utils        import make_whip_downwards

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    my_sim = Simulation( args )

    # Defining up the controller and objective
    if   args.ctrl_name == "joint_imp_ctrl": 

        # Define the controller 
        ctrl = JointImpedanceController( my_sim, args )

        if   ctrl.n_act == 2:
            ctrl.set_impedance( Kq = C.K_2DOF, Bq = 0.10 * C.K_2DOF )
            mov_arrs  = np.array(  [ -1.3327 , 0.17022, 1.5708 , 0.13575, 0.8011  ] )
                
        elif ctrl.n_act == 4:
            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )                   
            mov_arrs  = np.array(  [-0.944, 1.047, 0.026, 1.363, 1.729, -1.049, 0.013, 1.424, 0.583] )

        # Define the objective function
        # obj = DistFromTip2Target( my_sim.mj_model, my_sim.mj_data, args )
        obj = None

    elif args.ctrl_name == "task_imp_ctrl":
        pass

    else:
        raise ValueError( f"[ERROR] Wrong controller name" )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.set_obj( obj )

    n = my_sim.ctrl.n_act

    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( n ) }

    my_sim.ctrl.set_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time  )    
    my_sim.initialize( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Set the initial configuration of the whip downward    
    make_whip_downwards( my_sim )

    # Run the simulation
    my_sim.run( )

    print( f"The minimum value of this trial is { min( my_sim.obj_arr ):.5f}" )

    if args.is_save_data: 
        ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

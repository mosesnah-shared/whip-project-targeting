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
        ctrl = JointImpedanceController( my_sim, args, name = "joint_imp_1" )

        if   ctrl.n_act == 2:
            ctrl.set_impedance( Kq = C.K_2DOF, Bq = 0.10 * C.K_2DOF )
            n = my_sim.n_act

            mov_arrs  = np.array(  [ -1.3327,  0.17022, 1.5708 , 0.13575, 0.8011  ] )
            ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time  )    
            ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = np.array( [ -1.45, 0.3 ]), D = mov_arrs[ -1 ], ti = args.start_time + 0.4  )                

        elif ctrl.n_act == 4:
            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )                   
            mov_arrs  = np.array(  [-0.94248, 0.     , 0.     , 1.41372, 1.72788, 0.     , 0.     , 1.41372, 1.31667] )

            n = my_sim.n_act
            ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time  )                
            # ctrl.add_mov_pars( q0i = np.zeros( n ) , q0f = np.array( [ -1.45, 0.3, 0.2, 0.4 ]), D = mov_arrs[ -1 ], ti = args.start_time + 0.4  )                

        # Define the objective function
        obj = DistFromTip2Target( my_sim.mj_model, my_sim.mj_data, args, tol = 5 )
        # obj = None

    elif args.ctrl_name == "task_imp_ctrl":
        pass

    else:
        raise ValueError( f"[ERROR] Wrong controller name" )

    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.set_obj( obj )


    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Set the initial configuration of the whip downward    
    if "whip" in args.model_name: make_whip_downwards( my_sim )
    my_sim.forward( )

    # Run the simulation
    my_sim.run( )

    print( f"The minimum value of this trial is { min( my_sim.obj_arr ):.5f}" )

    if args.is_save_data: 
        ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

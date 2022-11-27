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
            ctrl.set_impedance( Kq = C.K_2DOF, Bq = 0.05 * C.K_2DOF )
            n = my_sim.n_act


            # mov_arrs  = np.array(  [-1.3614,  0.7854,  1.0472,  0.4712,  0.5833] )       # for N10
            # mov_arrs  = np.array(  [ -1.3614,  0.4712,  1.0472,  0.4712,  0.5833] )     # for N15
            # mov_arrs  = np.array(  [ -1.5320, 0.4593, 1.0486, 0.9931, 0.6100  ] )     # for N20
            # mov_arrs  = np.array(  [ -1.3614, 0.4712 ,  1.0472,  0.7854, 0.58338 ] )     # for N25

            mov_arrs = np.array( [0.7854, 0.4712, -1.3614, 0.1571,  1.0472, 0.4712, 0.5833, 1.4389, 0.3333 ] )

            init  = mov_arrs[     : n   ]
            mid   = mov_arrs[  n  : 2*n ]
            final = mov_arrs[ 2*n : 3*n ]
            D1 = mov_arrs[ -3 ]
            D2 = mov_arrs[ -2 ]
            toff = mov_arrs[ -1 ] * D1

            ctrl.add_mov_pars( q0i = init, q0f = mid, D = D1, ti = args.start_time  )    
            ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = final - mid, D = D2, ti = args.start_time + D1 + toff)    
            

        elif ctrl.n_act == 4:
            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.30 * C.K_4DOF )       
            
            n = my_sim.n_act

            # For Bq 0.2, 0.7057,  0.0000,  1.2549,  0.4712, -1.2461,  0.0043, -1.3532,  0.8358,  1.8960, -0.5258,  0.3491,  0.9431,  0.9304,  0.4626, -0.0214
            # For Bq 0.1, 0.7854, 0, 0.8378, 0.4712, -1.2566, 0, -0.8378, 0.1571, 2.4086, -1.0472, -0.3491, 0.4712, 0.9500, 0.5833, 0            


    
            mov_arrs = np.array( [ 0.7057,  0.0000,  1.2549,  0.4712, -1.2461,  0.0043, -1.3532,  0.8358,  1.8960, -0.5258,  0.3491,  0.9431,  0.9304,  0.4626, -0.0214] )

            init  = mov_arrs[     : n   ]
            mid   = mov_arrs[  n  : 2*n ]
            final = mov_arrs[ 2*n : 3*n ]
            D1 = mov_arrs[ -3 ]
            D2 = mov_arrs[ -2 ]
            toff = mov_arrs[ -1 ] * D1

            ctrl.add_mov_pars( q0i = init, q0f = mid, D = D1, ti = args.start_time  )    
            ctrl.add_mov_pars( q0i = np.zeros( n ), q0f = final - mid, D = D2, ti = args.start_time + D1 + toff)    
            

            n = my_sim.n_act
            # ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time  )                
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

    args.cam_pos = "2.4088397035938347 2.009765064162837 3.6319497605437387 0.041214400000000005 -53.40000000000003 -139.0000000000001"

    init_cond = { "qpos": mov_arrs[ :n ] ,  "qvel": np.zeros( n ) }
    my_sim.init( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Set the initial configuration of the whip downward    
    if "whip" in args.model_name: make_whip_downwards( my_sim )
    my_sim.forward( )

    # Run the simulation
    my_sim.run( )

    print( f"The minimum value of this trial is { min( my_sim.obj_arr[ : my_sim.n_steps ] ):.5f}" )

    if args.is_save_data: 
        ctrl.export_data( my_sim.tmp_dir )

    my_sim.close( )

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
import scipy.io
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

    # Target type
    idx = args.target_type
    args.model_name = "3D_model_w_whip_T" + str( idx )
    
    # idx -1 
    idx -= 1

    my_sim = Simulation( args )

    

    # Defining up the controller and objective
    if   args.ctrl_name == "joint_imp_ctrl": 

        # Define the controller 
        ctrl = JointImpedanceController( my_sim, args, name = "joint_imp_1" )

        if   ctrl.n_act == 2:
            ctrl.set_impedance( Kq = C.K_2DOF, Bq = 0.05 * C.K_2DOF )
            n = my_sim.n_act


            # mov_arrs  = np.array(  [ -1.3614,  0.7854,  1.0472,  0.4712,  0.5833] )       # for N10
            # mov_arrs  = np.array(  [ -1.3614,  0.4712,  1.0472,  0.4712,  0.5833] )     # for N15
            # mov_arrs  = np.array(  [ -1.5320,  0.4593,  1.0486,  0.9931, 0.6100  ] )     # for N20
            # mov_arrs  = np.array(  [ -1.3614,  0.4712,  1.0472,  0.7854, 0.58338 ] )     # for N25

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
            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )       
            
            n = my_sim.n_act


            mov_arrs = np.array( [ [ -1.3614 ,    0.0, -0.3491, 1.4137, 1.7279,     0.0,     0.0, 1.4137, 0.9500 ], 
                                   [ -0.9425 ,    0.0, -1.0472, 1.4137, 1.7279, -1.0472,     0.0, 0.4712, 0.9500 ],
                                   [ -0.9425 , 1.0472,     0.0, 1.4137, 1.7279, -1.0472,     0.0, 1.4137, 0.5833 ],
                                   [ -1.5475 ,    0.0, -0.3491, 1.4137, 1.7279,     0.0,     0.0, 0.3665, 0.9500 ], 
                                   [ -1.0821 , 1.0472,  1.0472, 0.8203, 1.7279, -1.0472,  1.3963, 0.1571, 0.9500 ], 
                                   [ -0.9425 , 1.0601,  0.3491, 0.9948, 1.7395, -0.9696, -0.2715, 0.9483, 0.5245 ] ] ) 

            cam_pos = [ "1.8484408987312249 -0.07357889449301756 -0.11709042487171689 2.231036477946396 -20.000000000000025 173.6", 
                        "2.244067989555753 2.113785561006945 0.4351389641609487 0.40798672746276177 -19.400000000000052 -137.60000000000016",
                        "0.8084722117980659 1.040155982419829 0.9681448511016364 2.2666834858125764 -51.800000000000054 -78.40000000000016",
                        "1.8484408987312249 -0.07357889449301756 -0.11709042487171689 2.231036477946396 -20.000000000000025 173.6", 
                        "2.244067989555753 2.113785561006945 0.4351389641609487 0.40798672746276177 -19.400000000000052 -137.60000000000016",
                        "0.8084722117980659 1.040155982419829 0.9681448511016364 2.2666834858125764 -51.800000000000054 -78.40000000000016" ]

            init  = mov_arrs[ idx,    : n   ]
            final = mov_arrs[ idx, n  : 2*n ]
            D = mov_arrs[ idx, -1 ]
            
            ctrl.add_mov_pars( q0i = init, q0f = final, D = D, ti = args.start_time  )    
            

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

    args.cam_pos = cam_pos[ idx ]

    init_cond = { "qpos": mov_arrs[ idx, :n ] ,  "qvel": np.zeros( n ) }
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

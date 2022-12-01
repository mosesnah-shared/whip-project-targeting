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
import scipy.io
import numpy      as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from datetime     import datetime
from simulation   import Simulation
from controllers  import JointImpedanceController
from objectives   import DistFromTip2Target
from constants    import my_parser
from constants    import Constants  as C
from utils        import make_whip_downwards, print_vars

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Generate the parser, which is defined 
parser = my_parser( )
args, unknown = parser.parse_known_args( )

if __name__ == "__main__":

    args.model_name = "3D_model_w_whip_T" + str( args.target_idx )

    # Generate an instance of our Simulation
    # The model is generated since the model name is passed via arguments
    my_sim = Simulation( args )

    # Define the controller 
    ctrl = JointImpedanceController( my_sim, args, name = "joint_imp_1" )

    if   ctrl.n_act == 2:
        ctrl.set_impedance( Kq = C.K_2DOF, Bq = 0.10 * C.K_2DOF )

    elif ctrl.n_act == 4:
        ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )              

    # Define the objective function
    obj = DistFromTip2Target( my_sim.mj_model, my_sim.mj_data, args, tol = 15 )


    # Add the controller and objective of the simulation
    my_sim.add_ctrl( ctrl )
    my_sim.set_obj( obj )


    L_arrs = { "init":[] , "final":[] , "D":[] }
    my_mov_arrs = { "init":[] , "final":[] , "D":[] }
    tmp_str =  [ "init", "final", "D" ]
    # tmp_str =  [ "D" ]

    iter = 0 

    mov_arrs = np.array( [ [ -1.3614 ,    0.0, -0.3491, 1.4137, 1.7279,     0.0,     0.0, 1.4137, 0.9500 ], 
                            [ -0.9425 ,    0.0, -1.0472, 1.4137, 1.7279, -1.0472,     0.0, 0.4712, 0.9500 ],
                            [ -0.9425 , 1.0472,     0.0, 1.4137, 1.7279, -1.0472,     0.0, 1.4137, 0.5833 ],
                            [ -1.5475 ,    0.0, -0.3491, 1.4137, 1.7279,     0.0,     0.0, 0.3665, 0.9500 ], 
                            [ -1.0821 , 1.0472,  1.0472, 0.8203, 1.7279, -1.0472,  1.3963, 0.1571, 0.9500 ], 
                            [ -0.9425 , 1.0601,  0.3491, 0.9948, 1.7395, -0.9696, -0.2715, 0.9483, 0.5245 ] ] ) 

    mov_arr = mov_arrs[ args.target_idx - 1, : ]

    n = my_sim.n_act

    init  = mov_arr[   : n   ]
    final = mov_arr[ n : 2*n ]
    D     = mov_arr[ -1 ]

    lb  = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )               
    ub  = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] ) 

    ub_init  = ub[ :n ]
    ub_final = ub[ n:2*n ]
    ub_D = ub[ -1 ]

    lb_init  = lb[ :n ]
    lb_final = lb[ n:2*n ]
    lb_D = lb[ -1 ]

    for name in tmp_str:

        for iter in range( 200 ):
            # Reset the simulation 
            my_sim.reset( )            

            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )   

            # Add noise
            noise1 = np.random.uniform( -0.05*( ub_init  - lb_init  ), 0.05*( ub_init  - lb_init ) , 4 ) if name == "init"  else np.zeros( 4 )
            noise2 = np.random.uniform( -0.05*( ub_final - lb_final ), 0.05*( ub_final - lb_final ), 4 ) if name == "final" else np.zeros( 4 )
            noise3 = np.random.uniform( -0.05*( ub_D - lb_D ),0.05*( ub_D - lb_D ), 1 )          if name == "D"     else np.zeros( 1 )

            ctrl.add_mov_pars( q0i = mov_arr[ :n ] + noise1, q0f = mov_arr[ n:2*n ] + noise2, D = mov_arr[ -1 ] + noise3, ti = args.start_time  )                
            my_sim.init( qpos = mov_arr[ :n ] + noise1, qvel = np.zeros( n ) )

            # Set the initial configuration of the whip downward    
            make_whip_downwards( my_sim )
            my_sim.forward( )

            # Run the simulation
            my_sim.run( )
            
            print_vars( { "Iteration": iter + 1, "vals": mov_arr + np.concatenate( [ noise1, noise2, noise3 ] ), "opt_vals" : min( my_sim.obj_arr[ : my_sim.n_steps ] ) } )
            iter += +1

            my_mov_arrs[ name ].append( mov_arr + np.concatenate( [ noise1, noise2, noise3 ] ) )
            L_arrs[ name ].append( min( my_sim.obj_arr[ : my_sim.n_steps ] ) )

    dir_name  = C.SAVE_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S" )
    os.mkdir( dir_name )  
    file_name = dir_name + "/" + str( args.target_idx ) + "sensitivity_analysis.mat"

    scipy.io.savemat( file_name, { "init_output": L_arrs[ "init" ], "final_output": L_arrs[ "final" ], "D_output":L_arrs[ "D" ], 
                                   "init_input_pars": my_mov_arrs[ "init" ] , "final_input_pars": my_mov_arrs[ "final" ], "D_input_pars": my_mov_arrs[ "D" ] } )

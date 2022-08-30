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

    if args.target_idx == 1:     
        args.model_name = "3D_model_w_whip_T1"

    elif args.target_idx == 2:
        args.model_name = "3D_model_w_whip_T2"

    elif args.target_idx == 3:
        args.model_name = "3D_model_w_whip_T3"

    elif args.target_idx == 4:     
        args.model_name = "3D_model_w_whip_T4"

    elif args.target_idx == 5:
        args.model_name = "3D_model_w_whip_T5"

    elif args.target_idx == 6:
        args.model_name = "3D_model_w_whip_T6"

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

            if args.target_idx == 1:     
                mov_arrs  = np.array(  [-1.3540, 0.0020,  -0.2817, 1.4156, 1.7279, -0.0817, -0.0163, 1.3788, 0.9500] )
    
            elif args.target_idx == 2:
                mov_arrs  = np.array(  [-0.9425, 0.0980,  -1.0492, 1.3641, 1.7279, -1.0492, -0.0184, 0.4712, 0.9500] )

            elif args.target_idx == 3:
                mov_arrs  = np.array(  [-0.9442, 1.0472,   0.0259, 1.3633, 1.7292, -1.0486,  0.0129, 1.4241, 0.5833] )

            elif args.target_idx == 4:     
                mov_arrs  = np.array(  [-1.50098, 0.     ,-0.2715 , 1.41372, 1.72788, 0.     , 0.     , 0.36652, 0.95   ] )
    
            elif args.target_idx == 5:
                mov_arrs  = np.array(  [-1.0821 , 1.0472 , 1.0472 , 0.7854 , 1.72788,-1.0472 , 1.39626, 0.12217, 0.95   ] )

            elif args.target_idx == 6:
                mov_arrs  = np.array(  [-0.94248, 1.0472 , 0.34907, 1.09956, 1.72788,-1.0472 ,-0.23271, 1.06465, 0.58333] )
            else:
                raise NotImplementedError( )                

            n = my_sim.n_act
            ctrl.add_mov_pars( q0i = mov_arrs[ :n ], q0f = mov_arrs[ n:2*n ], D = mov_arrs[ -1 ], ti = args.start_time  )                

            # For target 4, 5, 6, the optimal movement parameters are as follows:
            # Target 4 [-1.50098, 0.     ,-0.2715 , 1.41372, 1.72788, 0.     , 0.     , 0.36652, 0.95   ]
            # Target 5 [-1.0821 , 1.0472 , 1.0472 , 0.7854 , 1.72788,-1.0472 , 1.39626, 0.12217, 0.95   ]
            # Target 6 [-0.94248, 1.0472 , 0.34907, 1.09956, 1.72788,-1.0472 ,-0.23271, 1.06465, 0.58333]

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


    L_arrs = { "init":[] , "final":[] , "D":[] }
    my_mov_arrs = { "init":[] , "final":[] , "D":[] }
    # tmp_str =  [ "init", "final", "D" ]
    tmp_str =  [ "D" ]

    iter = 0 

    for name in tmp_str:
        for noise in np.arange( -0.2, +0.2,  0.002 ):
  
            # Reset the simulation 
            my_sim.reset( )

            ctrl.set_impedance( Kq = C.K_4DOF, Bq = 0.05 * C.K_4DOF )         
            
            if args.target_idx == 1:     
                mov_arrs  = np.array(  [-1.3540, 0.0020,  -0.2817, 1.4156, 1.7279, -0.0817, -0.0163, 1.3788, 0.9500] )
    
            elif args.target_idx == 2:
                mov_arrs  = np.array(  [-0.9425, 0.0980,  -1.0492, 1.3641, 1.7279, -1.0492, -0.0184, 0.4712, 0.9500] )

            elif args.target_idx == 3:
                mov_arrs  = np.array(  [-0.9442, 1.0472,   0.0259, 1.3633, 1.7292, -1.0486,  0.0129, 1.4241, 0.5833] )

            elif args.target_idx == 4:
                mov_arrs  = np.array(  [-1.50098, 0.     ,-0.2715 , 1.41372, 1.72788, 0.     , 0.     , 0.36652, 0.95   ] )

            elif args.target_idx == 5:
                mov_arrs  = np.array(  [-1.0821 , 1.0472 , 1.0472 , 0.7854 , 1.72788,-1.0472 , 1.39626, 0.12217, 0.95   ] )

            elif args.target_idx == 6:
                mov_arrs  = np.array(  [-0.94248, 1.0472 , 0.34907, 1.09956, 1.72788,-1.0472 ,-0.23271, 1.06465, 0.58333] )
            else:
                raise NotImplementedError( )

            # For target 4, 5, 6, the optimal movement parameters are as follows:

            n = my_sim.n_act

            # Add noise
            noise1 = np.random.uniform( -.1, .1, 4 ) if name == "init"  else np.zeros( 4 )
            noise2 = np.random.uniform( -.1, .1, 4 ) if name == "final" else np.zeros( 4 )
            # noise3 = np.random.uniform( -.1, .1, 1 ) if name == "D"     else np.zeros( 1 )
            noise3 = np.array( [ noise ] )

            mov_arrs_w_noise = mov_arrs + np.concatenate( ( noise1, noise2, noise3 ) )

            ctrl.add_mov_pars( q0i = mov_arrs_w_noise[ :n ], q0f = mov_arrs_w_noise[ n:2*n ], D = mov_arrs_w_noise[ -1 ], ti = args.start_time  )                

            ctrl.set_impedance( Kq = C.K_dict[ n ], Bq = 0.05 * C.K_dict[ n ] )
            my_sim.init( qpos = mov_arrs[ :n ], qvel = np.zeros( n ) )

            # Set the initial configuration of the whip downward    
            make_whip_downwards( my_sim )

            # Run the simulation
            my_sim.run( )
            
            print_vars( { "Iteration": iter + 1, "vals": mov_arrs_w_noise[ : ], "opt_vals" : min( my_sim.obj_arr[ : my_sim.n_steps ] ) } )
            iter += +1

            my_mov_arrs[ name ].append( mov_arrs_w_noise )
            L_arrs[ name ].append( min( my_sim.obj_arr[ : my_sim.n_steps ] ) )
 

    dir_name  = C.SAVE_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S" )
    os.mkdir( dir_name )  
    file_name = dir_name + "/" + str( args.target_idx ) + "sensitivity_analysis.mat"

    scipy.io.savemat( file_name, { "init_output": L_arrs[ "init" ], "final_output": L_arrs[ "final" ], "D_output":L_arrs[ "D" ], 
                                   "init_input_pars": my_mov_arrs[ "init" ] , "final_input_pars": my_mov_arrs[ "final" ], "D_input_pars": my_mov_arrs[ "D" ] } )

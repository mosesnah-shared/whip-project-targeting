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
import nlopt
import scipy.io
import numpy  as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from datetime     import datetime
from simulation   import Simulation
from controllers  import JointImpedanceController
from objectives   import DistFromTip2Target
from constants    import my_parser
from constants    import Constants  as C
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       

# Generate the parser, which is defined in utils.py
parser = my_parser( )
args, unknown = parser.parse_known_args( )

# Generate an instance of our Simulation
my_sim = Simulation( args )

# Define the controller 
my_ctrl = JointImpedanceController( my_sim, args, name = "joint_imp_1" )

# Define the objective function
tol = 15
obj = DistFromTip2Target( my_sim.mj_model, my_sim.mj_data, args, tol  )

# Setup the controller and objective of the simulation
my_sim.add_ctrl( my_ctrl )
my_sim.set_obj( obj )

# iteration, optimal value and input parameters
iter_arr = []
opt_val_arr = []
input_par_arr = []

if __name__ == "__main__":

    if   my_ctrl.n_act == 2:
        #                           INIT                        MID                     FINAL                 D1      D2        alpha
        lb = np.array( [  0.0 * np.pi,           0.0, -0.5 * np.pi,         0.0, -0.5 * np.pi,           0.0, 0.4,    0.4,    -0.5 ] )     
        ub = np.array( [  0.5 * np.pi,   0.9 * np.pi, -0.1 * np.pi, 0.9 * np.pi,  0.5 * np.pi,   0.9 * np.pi, 1.5,    1.5,    +0.5 ] )     
        nl_init = ( lb + ub ) * 0.5 
        n_opt = 9        

    elif my_ctrl.n_act == 4:
        lb  = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )               
        ub  = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )              
        nl_init = ( lb + ub ) * 0.5         
        n_opt = 9

    else:
        raise NotImplementedError( )

    opt_type = nlopt.GN_DIRECT_L
    # opt_type = nlopt.GN_CRS2_LM

    opt = nlopt.opt( opt_type, n_opt )  
    iter = 0 

    n = my_ctrl.n_act    

    def nlopt_objective( pars, grad ):                              
        """
            The objective function of the nlopt function    
            We need to define and feed this into the nlopt optimization. 
        """

        n = my_ctrl.n_act

        # Reset the simulation 
        my_sim.reset( )

        init  = pars[    :   n ]
        final = pars[  n : 2*n ]
        D = pars[ -1 ]

        my_ctrl.add_mov_pars( q0i = init, q0f = final, D = D, ti = args.start_time  )  
        my_ctrl.set_impedance( Kq = C.K_dict[ n ], Bq = 0.05 * C.K_dict[ n ] )    
        
        my_sim.init( qpos = pars[ :n ], qvel = np.zeros( n ) )

        # Set the initial configuration of the whip downward    
        make_whip_downwards( my_sim )

        # Run the simulation

        my_sim.run( )

        print_vars( { "Iteration": opt.get_numevals( ) + 1, "mov_pars": pars[ : ], "opt_vals" : min( my_sim.obj_arr[ : my_sim.n_steps ] ) } )

        iter_arr.append( opt.get_numevals( ) + 1 )
        input_par_arr.append( np.copy( pars[ : ] ) )
        opt_val_arr.append( min( my_sim.obj_arr[ : my_sim.n_steps ] ) )

        return min( my_sim.obj_arr[ : my_sim.n_steps ] )

    opt.set_lower_bounds( lb )
    opt.set_upper_bounds( ub )
    opt.set_maxeval( 600 )

    opt.set_min_objective( nlopt_objective )
    opt.set_stopval( 1e-5 ) 

    xopt = opt.optimize( nl_init )

    print_vars( { "Optimal Values": xopt[ : ], "Result": opt.last_optimum_value( ) }  )

    # Saving the values in tmp
    # Generate tmp directory before moving to results folder 
    dir_name  = C.SAVE_DIR + datetime.now( ).strftime( "%Y%m%d_%H%M%S" )
    os.mkdir( dir_name )  
    file_name = dir_name + "/optimization.mat"

    scipy.io.savemat( file_name, { "iter": iter_arr, "opt_val": opt_val_arr, "input_pars" :input_par_arr, "Kq": my_ctrl.Kq, "Bq": my_ctrl.Bq } )

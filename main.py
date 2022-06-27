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
import argparse

import nlopt
import numpy as np

# To Add Local Files, adding the directory via sys module
# __file__ saves the current directory of this file. 
sys.path.append( os.path.join( os.path.dirname(__file__), "modules" ) )

from simulation   import Simulation
from controllers  import JointImpedanceController
from objectives   import DistFromTip2Target
from constants    import Constants  as C
from utils        import *

# Setting the numpy print options, useful for printing out data with consistent pattern.
np.set_printoptions( linewidth = np.nan, suppress = True, precision = 4 )       
                                                                                
# Argument Parsers
parser = argparse.ArgumentParser( description = 'Parsing the arguments for running the simulation' )
parser.add_argument( '--version'     , action = 'version'     , version = C.VERSION )

parser.add_argument( '--start_time'  , action = 'store'       , type = float ,  default = 0.0,                   help = 'Start time of the controller'                                                      )
parser.add_argument( '--run_time'    , action = 'store'       , type = float ,  default = 4.0,                   help = 'Total run time of the simulation'                                                  )
parser.add_argument( '--model_name'  , action = 'store'       , type = str   ,  default = '2D_model' ,           help = 'Model name for the simulation'                                                     )
parser.add_argument( '--ctrl_name'   , action = 'store'       , type = str   ,  default = 'joint_imp_ctrl',      help = 'Model name for the simulation'                                                     )
parser.add_argument( '--cam_pos'     , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the camera position'                                         )
parser.add_argument( '--mov_pars'    , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the movement parameters'                                     )
parser.add_argument( '--target_type' , action = 'store'       , type = int   ,                                   help = 'Save data log of the simulation, with the specified frequency'                     )
parser.add_argument( '--print_mode'  , action = 'store'       , type = str   ,  default = 'normal',              help = 'Print mode, choose between [short] [normal] [verbose]'                             )
parser.add_argument( '--print_freq'  , action = 'store'       , type = int   ,  default = 60      ,              help = 'Specifying the frequency of printing the date.'                                    )
parser.add_argument( '--vid_speed'   , action = 'store'       , type = float ,  default = 1.      ,              help = 'The speed of the video. It is the gain of the original speed of the video '        )

parser.add_argument( '--record_vid'  , action = 'store_true'  ,                                                  help = 'Record video of the simulation,  with the specified speed'     )
parser.add_argument( '--save_data'   , action = 'store_true'  ,                                                  help = 'Save the details of the simulation'                            )
parser.add_argument( '--vid_off'     , action = 'store_true'  ,                                                  help = 'Turn off the video'                                            )
parser.add_argument( '--run_opt'     , action = 'store_true'  ,                                                  help = 'Run optimization of the simulation'                            )

# For jupyter compatibility.
# [REF] https://stackoverflow.com/questions/48796169/how-to-fix-ipykernel-launcher-py-error-unrecognized-arguments-in-jupyter
args, unknown = parser.parse_known_args( )

def run_single_trial( mj_sim, mov_pars: dict, init_cond: dict ):
    """
        A function for running a single trial of simulation and return an array of the objective value of the simulation. 
    """

    mj_sim.ctrl.set_traj( mov_pars = mov_pars )    
    mj_sim.initialize( qpos = init_cond[ "qpos" ], qvel = init_cond[ "qvel" ] )

    # Set the initial configuration of the whip downward    
    if "whip" in mj_sim.args.model_name: make_whip_downwards( mj_sim )

    # Run the simulation
    mj_sim.run( )

    # Return the value of the simulation's objective value 
    return mj_sim.obj_arr 


def run_nlopt( mj_sim, alg_type = nlopt.GN_DIRECT_L, max_trial: int = 600 ):
    """
        Running the optimization. 

        Arguments 
        ---------
            alg_type: the type of algorithm that you will run for the nlopt optimization. 
                      Please check /Library/Frameworks/Python.framework/Versions/3.8/lib/python3.8/nlopt.py for the options. 
    """

    if   mj_sim.ctrl.n_act == 2:
        lb = np.array( [ -0.5 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )     
        ub = np.array( [  1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )     
        init = ( lb + ub ) * 0.5 
        n_opt = 5        

    elif mj_sim.ctrl.n_act == 4:
        lb  = np.array( [ -0.5 * np.pi, -0.5 * np.pi, -0.5 * np.pi,           0, 0.1 * np.pi,  -0.5 * np.pi, -0.5 * np.pi,         0.0, 0.4 ] )               
        ub  = np.array( [ -0.1 * np.pi,  0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.0 * np.pi,   0.5 * np.pi,  0.5 * np.pi, 0.9 * np.pi, 1.5 ] )              
        init = ( lb + ub ) * 0.5         
        n_opt = 9        

    
    def nlopt_objective( pars, grad ):                              
        """
            The objective function of the nlopt function    
            We need to define and feed this into the nlopt optimization. 
        """
        n = my_sim.ctrl.n_act

        obj_arr = run_single_trial( my_sim,  mov_pars = {  "q0i": pars[ 0 : n ],  "q0f": pars[ n : 2 * n ],  "D": pars[ -1 ]  } , init_cond = { "qpos": pars[ 0 : n ], "qvel": np.zeros( n )  } )

        mj_sim.reset( )

        print_vars( { "Iteration": opt.get_numevals( ) + 1, "mov_pars": pars[ : ], "opt_vals" : min( obj_arr ) } )

        return min( obj_arr )

    opt = nlopt.opt( alg_type, n_opt )  

    opt.set_lower_bounds( lb )
    opt.set_upper_bounds( ub )
    opt.set_maxeval( max_trial )

    opt.set_min_objective( nlopt_objective )
    opt.set_stopval( 1e-8 ) 

    xopt = opt.optimize( init )

    print_vars( { "Optimal Values": xopt[ : ], "Result": opt.last_optimum_value( ) }  )

if __name__ == "__main__":

    # Generate an instance of our Simulation
    my_sim = Simulation( args )

    # Instance for the controller of the simulation
    if   args.ctrl_name == "joint_imp_ctrl": 
        ctrl = JointImpedanceController( my_sim.mj_model, my_sim.mj_data, args, t_start = args.start_time )

    elif args.ctrl_name == "task_imp_ctrl":
        pass

    # If the controller uses an machine learning algorithm - deep deterministic policy gradient (DDPG)
    elif args.ctrl_name == "ML_DDPG":
        pass

    # If the controller uses an machine learning algorithm - twin-delayed deep deterministic policy gradient (TD-3)
    elif args.ctrl_name == "ML_TD3":
        pass

    else:
        raise ValueError( f"[ERROR] Wrong controller name" )

    # Instance for the objective of the simulation 
    obj = DistFromTip2Target( my_sim.mj_model, my_sim.mj_data, args )

    # Setup the controller and objective of the simulation
    my_sim.set_ctrl( ctrl )
    my_sim.set_obj( obj )

    # Run Optimization
    if args.run_opt:
        run_nlopt( my_sim, max_trial = 3 )    

    else:

        # For a 2DOF model
        if   my_sim.ctrl.n_act == 2:
            
            mov_pars  = {  "q0i": np.array( [ .3, .4 ] ),   "q0f": np.array( [ .8, .5 ] ),          "D": 1.  } 
            init_cond = { "qpos": np.array( [ .3, .4 ] ),  "qvel": np.zeros( my_sim.ctrl.n_act ) }

        # For a 4DOF model
        elif my_sim.ctrl.n_act == 4:
            mov_pars  = {  "q0i": np.array( [ .3, .4, .4, .3 ] ),   "q0f": np.array( [ .6, .5, .7, .1 ] ),          "D": 1.  } 
            init_cond = { "qpos": np.array( [ .3, .4, .4, .3 ] ),  "qvel": np.zeros( my_sim.ctrl.n_act )  }            

        obj_arr = run_single_trial( my_sim,  mov_pars = mov_pars, init_cond = init_cond )
        print( f"The minimum value of this trial is { min(obj_arr):.5f}" )

        

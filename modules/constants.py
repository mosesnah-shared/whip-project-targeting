import argparse
import numpy as np

def my_parser( ):
    # Argument Parsers
    parser = argparse.ArgumentParser( description = 'Parsing the arguments for running the simulation' )
    parser.add_argument( '--version'     , action = 'version'     , version = Constants.VERSION )

    parser.add_argument( '--start_time'  , action = 'store'       , type = float ,  default = 0.0,                   help = 'Start time of the controller'                                                      )
    parser.add_argument( '--run_time'    , action = 'store'       , type = float ,  default = 4.0,                   help = 'Total run time of the simulation'                                                  )
    parser.add_argument( '--model_name'  , action = 'store'       , type = str   ,  default = '2D_model' ,           help = 'Model name for the simulation'                                                     )
    parser.add_argument( '--ctrl_name'   , action = 'store'       , type = str   ,  default = 'joint_imp_ctrl',      help = 'Model name for the simulation'                                                     )
    parser.add_argument( '--cam_pos'     , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the camera position'                                         )
    parser.add_argument( '--mov_pars'    , action = 'store'       , type = str   ,                                   help = 'Get the whole list of the movement parameters'                                     )
    parser.add_argument( '--target_type' , action = 'store'       , type = int   ,                                   help = 'Save data log of the simulation, with the specified frequency'                     )
    parser.add_argument( '--opt_type'    , action = 'store'       , type = str   ,  default = "nlopt" ,              help = '[Options] "nlopt", "ML_DDPG", "ML_TD3" '                                           )
    parser.add_argument( '--print_mode'  , action = 'store'       , type = str   ,  default = 'normal',              help = 'Print mode, choose between [short] [normal] [verbose]'                             )
    parser.add_argument( '--print_freq'  , action = 'store'       , type = int   ,  default = 60      ,              help = 'Specifying the frequency of printing the data.'                                    )
    parser.add_argument( '--save_freq'   , action = 'store'       , type = int   ,  default = 60      ,              help = 'Specifying the frequency of saving the data.'                                      )
    parser.add_argument( '--vid_speed'   , action = 'store'       , type = float ,  default = 1.      ,              help = 'The speed of the video. It is the gain of the original speed of the video '        )

    parser.add_argument( '--record_vid'  , action = 'store_true'  , dest = "is_record_vid"  ,                        help = 'Record video of the simulation,  with the specified speed'     )
    parser.add_argument( '--save_data'   , action = 'store_true'  , dest = "is_save_data"   ,                        help = 'Save the details of the simulation'                            )
    parser.add_argument( '--vid_off'     , action = 'store_true'  , dest = "is_vid_off"     ,                        help = 'Turn off the video'                                            )
    parser.add_argument( '--run_opt'     , action = 'store_true'  , dest = "is_run_opt"     ,                        help = 'Run optimization of the simulation'                            )

    return parser


class Constants:
    PROJECT_NAME            = '[M3X] Whip Project'
    VERSION                 = '2.0.0'
    UPDATE_DATE             = '2022.06.21'
    AUTHOR_GITHUB           = 'mosesnah-shared'
    AUTHOR_FULL_NAME        = 'Moses C. Nah'
    DESCRIPTION             = "mujoco-py scripts for running a whip-targeting simuation"
    URL                     = 'https://github.com/mosesnah-shared/whip-project-targeting',
    AUTHOR_EMAIL            = 'mosesnah@mit.edu', 'mosesnah@naver.com'

    # =============================================================== #
    # Constant variables for running the simulation
    # =============================================================== #
    
    # The module directory which contains all python modules.
    MODULE_DIR     = "./modules/"

    # The model directory which contains all the xml model files.
    MODEL_DIR     = "./models/"

    # The directory which saves all the simulation results
    SAVE_DIR      = "./results/"

    # The directory which saves all the temporary data
    TMP_DIR       = "./tmp/"

    # The constant K matrices that we will use often. 
    # For a 2DOF model
    # [REF] Nah, Moses C., et al. "Dynamic primitives facilitate manipulating a whip.", BIOROB 2020
    K_2DOF = np.array( [ [ 29.50, 14.30 ], [ 14.30, 39.30 ] ] )
    
    # For a 4DOF model.
    # [REF] Manipulating a whip in 3D
    K_4DOF = np.array( [ [ 17.40, 6.85, -7.75, 8.40 ] ,
                         [  6.85, 33.0,  3.70, 0.00 ] ,
                         [ -7.75, 3.70,  27.7, 0.00 ] ,
                         [  8.40, 0.00,  0.00, 23.2 ] ] )

    # K_4DOF = 300 * np.eye( 4 )

    # The above value is the symmetric part of the following matrix
    # K_4DOF = np.array( [ [ 17.40, 4.70, -1.90, 8.40 ] ,
    #                      [  9.00, 33.0,  4.40, 0.00 ] ,
    #                      [ -13.6, 3.00,  27.7, 0.00 ] ,
    #                      [  8.40, 0.00,  0.00, 23.2 ] ] )

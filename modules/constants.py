import numpy as np

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

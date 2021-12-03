import mujoco_py as mjPy
import numpy     as np

from modules.constants       import Constants as C
from modules.utils           import my_print
from scipy.spatial.transform import Rotation as R



def main( ):

    model_name = "1_3D_model_update.xml"

    mjModel  = mjPy.load_model_from_path( C.MODEL_DIR + model_name )      #  Loading xml model as and save it as "model"
    mjSim    = mjPy.MjSim( mjModel )                                    # Construct the simulation environment and save it as "sim"
    mjData   = mjSim.data                                               # Construct the basic MuJoCo data and save it as "mjData"
    mjViewer = mjPy.MjViewerBasic( mjSim )

    run_time    = 100
    t           = 0
    sim_step    = 0
    update_rate = 60
    dt          = mjModel.opt.timestep

    while t <= run_time:

        if sim_step % update_rate == 0:

            mjViewer.render( )                                     # Render the simulation

        mjSim.forward( )
        # mjSim.step( )

        t += dt

        sim_step += 1

        # Our goal is to
        # [#1] Get the rotation matrix of our current upper-limb
        # [#2] Change the rotation matrix to Quaternion.
        # [#3] Plug
        # -1.51650,      0,        0, 0.15708,
        r = R.from_euler( 'yxz', [1.51650, 0, 0], degrees=False )
        print( r.as_quat( ) )

        # Gravity Torque for the compensation
        # print("HI", mjData.get_site_jacp( "site_upper_arm_COM" ) )

        G = np.dot( mjData.get_site_jacp( "site_upper_arm_COM" ).reshape( 3, -1 )[ :, 0 : 4 ].T, - 1.595 * mjModel.opt.gravity  )  \
          + np.dot( mjData.get_site_jacp(  "site_fore_arm_COM" ).reshape( 3, -1 )[ :, 0 : 4 ].T, - 0.869 * mjModel.opt.gravity )

        print( "1", mjData.get_site_jacp(  "site_fore_arm_COM" ).reshape( 3, -1 ) )
        print( "2", mjData.get_site_jacp( "site_upper_arm_COM" ).reshape( 3, -1 ) )

        print( "Gravity Torque", G )


        my_print(  timeStep = t )

        mjData.qpos[ 0:4 ] = r.as_quat( )
        mjData.qpos[ -1]   = 0.15708

        print( "helo", mjData.qpos )        # For the shoulder, we have four values, which are quaternion form.
                                    # [1 0 0 0], means a + bi + cj + dk
        print( "Hi", np.linalg.norm( mjData.qpos[ 0 : 4 ], 2 )  )


        # [-0.29356674  0.         -0.01595528]
        # elbow position [-0.29356674  0.         -0.01595528]
        print( "elbow position", mjData.geom_xpos[ mjModel.geom_name2id( "geom_elbow" ) ] )

if __name__ == "__main__":
    main(  )

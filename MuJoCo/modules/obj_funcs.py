import numpy as np

try:
    import mujoco_py as mjPy
except ImportError as e:
    raise error.DependencyNotInstalled( "{}. (HINT: you need to install mujoco_py, \
                                             and also perform the setup instructions here: \
                                             https://github.com/openai/mujoco-py/.)".format( e ) )

def dist_from_tip2target( mjModel, mjData ):
    """
        Getting the distance between the tip of the whip and target.

        In the .xml model file, The geometry name of target is named as "target",
        and the geometry name of the tip of the whip contains "tip" within the name.
    """

    def get_dist( geom_name1, geom_name2 ):                                     # Getting the distance between two geometry.
        return np.linalg.norm( mjData.get_geom_xpos( geom_name1 ) \
                             - mjData.get_geom_xpos( geom_name2 ), ord = 1  )

    # Getting the geometry name of the tip of the whip.
    if not isinstance( mjModel, mjPy.cymj.PyMjModel ) or not isinstance( mjData, mjPy.cymj.PyMjData ):
        raise ValueError( "Wrong input for function" )

    tip_geom_name    = [ s for s in mjModel.geom_names[ : ] if "tip"    in s.lower() ]  # Find the geometry containing "tip"    for the geom name.
    target_geom_name = [ s for s in mjModel.geom_names[ : ] if "target" in s.lower() ]  # Find the geometry containing "target" for the geom name.

    if len( tip_geom_name ) != 1 or len( target_geom_name ) != 1:
        raise ValueError( "Multiple or No Target/Tip Exist, Please check the model file used for the simulation")

    return get_dist( tip_geom_name[ 0 ], target_geom_name[ 0 ] )


if __name__ == "__main__":
    pass

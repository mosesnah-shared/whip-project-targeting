import re
import numpy     as np

class Objective( ):

    def __init__( self, mj_model, mj_data, args ):
        self.mj_model = mj_model
        self.mj_data  = mj_data
        self.args     = args

    def output_calc( self ):
        raise NotImplementedError  

class TargetState( Objective ):
    """ Trying to reach some target State  """

    def __init__( self, mjModel, mjData, mjArgs ):
        super().__init__( mjModel, mjData, mjArgs )

        self.targets = []

    def set_target( self, boundary ):
        # There will be a variable in focus, we need to "probe" that variable.
        # The boundary dictionary consists as follows:
        # [1] Which variable, is it the qpos or the qvel?
        # [2] For the variable defined in [1], which specific index?
        # [3] target value for that value
        # [4] And tolerance for that, +- [3]'s value
        self.targets.append( boundary )

    def output_calc( self ):

        # Iterating through the targets
        val = 0.0
        for target in self.targets:
            tmp = getattr( self.mjData, target[ "which_var"  ] )                # which_var is either "qpos" or "qvel"
            tmp = tmp[ target[ "idx" ]  ]                                       # Getting the value with corresponding index, idx is either list or scalar

            tmp_v = np.sqrt( np.sum( np.square( tmp - target[ "target_val" ] ) ) )
            if tmp_v <= target[ "tolerance" ]:
                tmp_v = 0.0

            val += tmp_v

        return val

class DistFromTip2Target( Objective ):
    """
        Getting the distance between the tip of the whip and target.

        In the .xml model file, The geometry name of target is named as "target",
        and the geometry name of the tip of the whip contains "tip" within the name.

        Retriving the distance between the tip of the whip and target.

    """
    def __init__( self, mjModel, mjData, mjArgs, tol = 1 ):
        """
            Arguments
            ---------
            tol: (int) The number of geom that we are accounting for the targeting task.
                       if tol = 5, then we are considering from geom21~25 for the distance.

        """

        super().__init__( mjModel, mjData, mjArgs )
        self.tol         = tol

        self.tip_name    = self._find_tip(  )                                   # Number of sub-models of the whip
        self.target_name = "geom_target"
        self.target_idx  = self.mjModel.geom_name2id( self.target_name )
        self.target_size = max( self.mjModel.geom_size[ self.target_idx ] )     # Get the size number of the target
        self.N           = int( re.search( r'\d+', self.tip_name ).group() )

        # The list of geometry that we are extracting for the targeting task
        if tol == 1:
            self.geom_list = list( )
        else:
            self.geom_list   = [ "geom_" + str( self.N - self.tol + i + 1 )  for i in range( self.tol - 1 )  ]

        self.geom_list.append( self.tip_name )


    def output_calc( self ):
        """ Calculate the objective function, which is the minimum distance between parts of the whip (ref. tol variable) and target. """
        # self.geom_list = ["geom_23", "geom_22", "geom_21" ]
        lens = [ length_elem2elem( self.mjModel, self.mjData, geom, self.target_name ) for geom in self.geom_list  ]
        output = min( lens )

        # if target is hit by the whip, set output as output and change to green light!
        # if output <= self.target_size + 0.012:
        # if output <= self.target_size:
        if output <= self.target_size + 0.012:
            output = 0.0  # For tolerance.
            self.mjModel.geom_rgba[ self.target_idx ] = [ 0, 1, 0, 1 ]

        return output + 0.1     # If output 0.1 then 

    def _find_tip( self ):

        for name in self.mjModel.geom_names:
            if "tip" in name:
                return name

        raise ValueError( "No word with tip on the model, to use class DistFromTip2Target we need to have a geom with name tip"           )


if __name__ == "__main__":
    pass

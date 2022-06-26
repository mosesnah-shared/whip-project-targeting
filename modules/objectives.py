import numpy as np
from modules.utils import *

class ObjectiveFunction:
    """
        The objective function that we are planning to maximize/minimize 
    """

    def __init__( self, mj_model, mj_data, args ):
        self.mj_model = mj_model
        self.mj_data  = mj_data
        self.args     = args

        # This internal "func" will return a scalar value every time step 
        # The func should have the following arguments -- "mj_model", "mj_data" and "args". 
        self.func     = None

    def set_func( self, func ):
        """
            Define the function that will be called every time step. 
        """
        self.func = func

    def set_success_trig( self ):
        """
            This defines an act that will be used called when objective is successful. 
        """


    def output_calc( self ):

        return self.func( self.mj_model, self.mj_data, self.args  )

    def __add__( self, other_obj ):


        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args )
        new_obj.set_func(  self.func + other_obj.func   )

        return new_obj

    def __radd__( self, other_obj ):
        pass

    def __mul__( self, other_obj ):    
        pass

    def __rmul__( self, other_obj ):    
        pass


class DistFromTip2Target( ObjectiveFunction ):

    def __init__( self, mj_model, mj_data, args, tol = 1, func = min, is_indicate_hit = False ):
        """
            Arguments
            ---------
            tol: (int) The number of geom that we are accounting for the targeting task.
                       if tol = 5, then we are considering from geom21~25 for the distance.

            func: either min or max, since we 
        """
 
        super( ).__init__( mj_model, mj_data, args, func )
        self.tol = tol
        self.is_indicate_hit = is_indicate_hit 

        # Get the number of whip nodes 
        self.N =  sum( 'whip' in name for name in mj_model.body_names )

        # Get the index of the target, which is used often 
        self.idx_target = self.mj_model.geom_name2idx( "geom_target" )

        # Get the index of the tip of the whip
        self.idx_whip_tip = self.mj_model.geom_name2idx( "geom_whip_node" + str( self.N ) )

        # Get the size of the target, simply the value which is the biggest 
        self.target_size = max( self.mj_model.geom_size[ self.target_idx ] )

        # Get the size of the whip's node size
        self.whip_tip_size = max( self.mj_model.geom_size[ self.idx_whip_tip ] )

    def output_calc( self ):
        """ 
            Calculate the objective function, which is the distance between parts of the whip (ref. tol variable) and target. 
        """

        dists = [ get_length( self.mj_model, "geom", "target", "geom", "whip_node" + str( idx ) ) for idx in self.N - np.arange( 0, self.tol ) ]
        output = min( dists )

        # If target is hit by the tip of the whip
        if output <= self.target_size + self.whip_tip_size:

            # Set the output to be zero, since it is hit
            output = 0.0  

            # Set the color to Green light!
            if self.is_indicate_hit: self.mj_model.geom_rgba[ self.idx_target  ] = [ 0, 1, 0, 1 ]

        return output


class TipVelocity( ObjectiveFunction ):
    """
        Get the Euclidean norm of the tip velocity
    """
    def __init__( self, mj_model, mj_data, args ):
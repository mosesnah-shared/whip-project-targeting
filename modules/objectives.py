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

        # Point the output_func to the output_calc
        self.output_func = self.output_calc

    def output_calc( self, mj_model, mj_data, args ):
        """
            Define the function that will be called/calculated every time step. 

            The details of how to calculate the output values must be implemented in the subclasses

        """
        return self.output_func( mj_model, mj_data, args )

    def set_success_trig( self, cond ):
        """
            This defines an act that will be used called when objective is successful.  

        """
        # If condition is satisifed 
        # FILL IN THE DETAILS FOR THE CODE!
        pass


    # ============== Magic Methods ============== # 
    # These methods will help us write intuitive syntax. For instances
    # [Example] 2 * obj1 + obj2
    # =========================================== #

    def __add__( self, other_obj ):
        """
            +: Enables obj1 + obj2
        """        
        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: self.output_calc( model, data, args ) + other_obj.output_calc( model, data, args )
        return new_obj

    def __radd__( self, other_obj ):
        """
            + is a commutative operator, hence simply overwriting it with __add__
        """
        return self.__add__( other_obj )

    def __mul__( self, w: float ):    
        """
            Enables 3 * obj1 
        """        
        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: w * self.output_func( model, data, args ) 
        return new_obj

    def __rmul__( self, w: float ):    
        """
            A scalar multiplication is a commutative operator, hence simply overwriting it with __mul__
        """        
        return self.__mul__( w )

    def __truediv__( self, w: float ):
        """
            Enables obj1 / 2
        """        

        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: self.output_calc( model, data, args ) / w

        return new_obj    

    def __rtruediv__( self, w: float ):
        """
            Enables 2 / obj1
        """                

        new_obj = ObjectiveFunction( self.mj_model, self.mj_data, self.args  )
        new_obj.output_func = lambda model, data, args: w / self.output_calc( model, data, args ) 

        return new_obj    


class DistFromTip2Target( ObjectiveFunction ):

    def __init__( self, mj_model, mj_data, args, tol = 5, is_indicate_hit = False ):
        """
            Arguments
            ---------
            tol: (int) The number of geom that we are accounting for the targeting task.
                       if tol = 5, then we are considering from geom21~25 for the distance.

            func: either min or max, since we 
        """
 
        super( ).__init__( mj_model, mj_data, args )
        self.tol = tol
        self.is_indicate_hit = is_indicate_hit 

        # Get the number of whip nodes 
        self.N =  sum( 'whip' in name for name in mj_model.body_names )

        # Get the index of the target, which is used often 
        self.idx_target = self.mj_model.geom_name2id( "geom_target" )

        # Get the index of the tip of the whip
        self.idx_whip_tip = self.mj_model.geom_name2id( "geom_whip_node" + str( self.N ) )

        # Get the size of the target, simply the value which is the biggest 
        self.target_size = max( self.mj_model.geom_size[ self.idx_target ] )

        # Get the size of the whip's node size
        self.whip_tip_size = max( self.mj_model.geom_size[ self.idx_whip_tip ] )

    def output_calc( self, mj_model, mj_data, args ):
        """ 
            Calculate the objective function, which is the distance between parts of the whip (ref. tol variable) and target. 
            
            [Notes] [Moses C. Nah]
            While arguments passed to this function is strictly not used, we should keep it for using the magic methods of the parent class.
        """

        dists = [ get_length( mj_model, mj_data, "geom", "target", "geom", "whip_node" + str( idx ) ) for idx in self.N - np.arange( 0, self.tol ) ]

        output = min( dists )

        # If target is hit by the tip of the whip
        if output <= self.target_size + self.whip_tip_size:

            # Set the output to be zero, since it is hit
            output = 0.0  

            # Set the color to Green light!
            # if self.is_indicate_hit: mj_model.geom_rgba[ self.idx_target  ] = [ 0, 1, 0, 1 ]
            mj_model.geom_rgba[ self.idx_target  ] = [ 0, 1, 0, 1 ]

        return output

class TipVelocity( ObjectiveFunction ):
    """
        Get the Euclidean norm of the tip velocity
    """
    def __init__( self, mj_model, mj_data, args ):
        pass

    def output_calc( self ):
        pass
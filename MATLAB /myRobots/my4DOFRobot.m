classdef my4DOFRobot < my2DOFRobot
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an 4-DOF planar robot 
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     18-October-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
    end
        
    properties ( SetAccess = public )

        
    end
    
    methods (Static)

  
    end            
 

    methods
        function obj = my4DOFRobot( varargin ) %
                        
            obj.nDOF     = 4;                
            obj.idx_limb = [ 3, 4 ];                                       % The index of the frame which describes the limb frame
        end

        function setFrameSE3( obj )
            % Setting the SE(3) information of the frames. 
            % THIS PROCESS IS NECESSARY FOR ALL CALCULATION
            % Defining the se(3) matrix for each coordinate transformation        
            % For details of the frame coordinate information, please refer to the following paper 
            % [REF] Nah, Moses C. Dynamic primitives facilitate manipulating a whip. Diss. Massachusetts Institute of Technology, 2020.
            % Fig. 3.4.
            
            % Defining the se(3) matrix for each coordinate transformation        
            T01 = obj.se3( -obj.q( 1 ), zeros( 3, 1 )         , @roty );           
            T12 = obj.se3( -obj.q( 2 ), zeros( 3, 1 )         , @rotx );
            T23 = obj.se3(  obj.q( 3 ), zeros( 3, 1 )         , @rotz );
            T34 = obj.se3( -obj.q( 4 ), [ 0; 0; -obj.L( 1 ) ] , @roty );
            
            % [WARNING!!!]
            % We need to fill in the T_arr before we conduct .getM, .getC and .getG methods
            obj.T_arr = { T01, T01 * T12, T01 * T12 * T23,  T01 * T12 * T23 * T34 };
            
        end  

       
        

    end

        
end



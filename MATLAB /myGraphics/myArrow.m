classdef myArrow < handle
% % =============================================================== %
%   [DESCRIPTION]
%
%       myArrow class for defining a 3D Arrow Vector graphical object
%
% Construct an instance of the marker class
% [Inputs Arguments]
%      (1) x,y,z: The 1-by-N row vectors, x, y and z position where the vector is pointing w.r.t. the origin
%         
%      (2) orig: Origin of the vector. 3-by-N, Note that the x, y and z given is w.r.t the origin!
%       
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     08-June-2020    
% % =============================================================== %
    
    properties ( SetAccess = private )
        % [1] Interal Function to check the size
        isSizeSame = @( x,y,z ) ( ( length( x ) == length( y ) ) && ...
                                    length( x ) == length( z ) );          

    end

    properties ( SetAccess = public )
        name
        x 
        y
        z
        orig
        arrowColor
        arrowWidth
        arrowHeadSize
    end
    
    methods

        function obj = myArrow( x, y, z, orig, varargin )
            obj.x = x; obj.y = y; obj.z = z;            
            obj.orig = orig;
            
            r = myParser( varargin );                
            
            obj.arrowWidth    = r.arrowWidth;
            obj.arrowColor    = r.arrowColor;
            obj.arrowHeadSize = r.maxHeadSize;
                        
        end
        

    end
end


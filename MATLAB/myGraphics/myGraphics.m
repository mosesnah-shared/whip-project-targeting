classdef myGraphics < handle & matlab.mixin.Heterogeneous
% % =============================================================== %
%   [DESCRIPTION]
%
%       Defining the primitive (parent) myGraphics class for my graphics objects.
%       Using matlab.mixin.Heterogeneous for using "myGraphics" as an object array.
%       [REF]https://www.mathworks.com/help/matlab/matlab_oop/subclassing-multiple-classes.html
%
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     31-Jan-2020   
% % =============================================================== %
    
    properties ( SetAccess = private )
        

    end


    properties ( SetAccess = public )
      
       name  = "g" + num2str( randi ( 2^20 ) );                           % Randomizing the name of this marker
       type
    end
    
    methods

        function obj = myGraphics(  )

        end
        
        function setAttr( obj, varargin )
            % Initialization of the class + parameters 
            % Separately defining the initialization method will be useful when inheriting classes. 
           
            % [TODO] [Moses C. Nah] [2021.01.31]
            % The key is case sensitive when using isprop function.
            % It might be beneficial if we can make it non-case sensitive.
            
            while ~isempty( varargin  )                                    % Iterating along the varargin
               
				key = varargin{ 1 }; 
                val = varargin{ 2 };
				varargin = varargin( 3: end );                             % Updating the varargin.
                
                if isprop( obj, key )                                      % Check whether the given key exist as a property
                    
                   obj.( key ) = val;                                      % If exist, then put value onto the key
                else 
                   error( "%s doesn't exist as a property", key  )
                   
                end
 
                
            end
            
        end

    end
end


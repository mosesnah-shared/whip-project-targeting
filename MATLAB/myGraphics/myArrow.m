classdef myArrow < myGraphics 
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
 

    end

    properties ( SetAccess = public )
        % [Properties]
        %   - The variables should match the name with the "quiver" function attributes 
        hfunc                                                              % The function handle for this graphic object.                
        
        % x,y,z,u,v,w position data of the marker
        XData
        YData
        ZData
        UData
        VData
        WData

        
        
        % Graphic attributes
        % Whole list of the graphic attributes, useful when we set the attributes and its values.
        gAttrList  = [ "LineWidth", "Color", "MaxheadSize", ...
                       "XData", "YData", "ZData", "UData", "VData", "WData" ]; 
                   
        % List of the graphic attributes that is updated for each time step.
        % Usually, the position datas should be updated
        gAttrUpdateList  = [ "XData", "YData", "ZData", "UData", "VData", "WData" ];                
        
        LineWidth   = 2;
        Color       = [0, 0.4470, 0.7410 ];
        MaxheadSize = 0.9;
        
    end
    
    methods

        function obj = myArrow( varargin )
            
            obj.type = "arrow";
            obj.setAttr( varargin{ : } )                                   % The initialize function is defined under "myGraphics.m" file.
  
        end
        
        function h = create( obj, hplot )
            % Creating the primitive object,
            % After calling this, we can set the graphical details.
            % Once this is created, we cannot change 2D to 3D, but 3D to 2D is still possible.
            if isempty( obj.ZData )

               h = quiver(   obj.XData( 1 ), obj.YData( 1 ), obj.UData( 1 ), obj.VData( 1 ), 'parent', hplot );

            else   
               
               h = quiver3(  obj.XData( 1 ), obj.YData( 1 ), obj.ZData( 1 ), ...
                             obj.UData( 1 ), obj.VData( 1 ), obj.WData( 1 ), 'parent', hplot );                
    
            end            
            
            % Setting the detailed graphical colors 
            for attr = obj.gAttrList 
                set( h, attr, obj.( attr ) );
            end
            
        end        
         

    end
end


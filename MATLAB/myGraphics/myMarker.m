classdef myMarker < myGraphics 
% % =============================================================== %
%   [DESCRIPTION]
%       myMarker class for defining a single marker in the plot
%
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
        %   - The variables should match the name with the "scatter" function attributes 
        hfunc                                                              % The function handle for this graphic object.                
        
        % x,y,z position data of the marker
        XData
        YData
        ZData
        
        % Graphic attributes
        % Whole list of the graphic attributes, useful when we set the attributes and its values.
        gAttrList  = [ "LineWidth", "SizeData", "Marker", "MarkerFaceColor", ...
                       "MarkerEdgeColor", "MarkerFaceAlpha", "MarkerEdgeAlpha" ]; 
                   
        % List of the graphic attributes that is updated for each time step.
        % Usually, the position datas should be updated
        gAttrUpdateList  = [ "XData", "YData", "ZData" ];                    
        
                   
        LineWidth       = 2;
        SizeData        = 625;
        Marker          = 'o';
        MarkerFaceColor = [1,      1,      1 ];
        MarkerEdgeColor = [0, 0.4470, 0.7410 ];
        MarkerFaceAlpha = 1;
        MarkerEdgeAlpha = 1;
        
    end
    
    methods

        function obj = myMarker( varargin )
            % Construct an instance of the marker class
            % [Inputs Arguments]
            %       varargin: The varargin should always get the value in 'key', 'val' pairs./
            %        example: 'name', '123', 'XData', [1,3], 'YData', [3,2]
            obj.type = "marker";
            
            obj.setAttr( varargin{ : } )                                   % The initialize function is defined under "myGraphics.m" file.
            
            
        end
        
        function h = create( obj, hplot )
            % Creating the primitive object,
            % After calling this, we can set the graphical details.
            % Once this is created, we cannot change 2D to 3D, but 3D to 2D is still possible.
            if isempty( obj.ZData )

               h = scatter(   obj.XData( 1 ), obj.YData( 1 ), 'parent', hplot );

            else   
               
               h = scatter3(  obj.XData( 1 ), obj.YData( 1 ), obj.ZData( 1 ), 'parent', hplot );                
                
            end            
            
            % Setting the detailed graphical colors 
            for attr = obj.gAttrList 
                set( h, attr, obj.( attr ) );
            end
            
        end
                
 
    end
end


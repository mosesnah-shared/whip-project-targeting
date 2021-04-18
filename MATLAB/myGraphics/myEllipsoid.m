classdef myEllipsoid < myGraphics 
% % =============================================================== %
%   [DESCRIPTION]
%
%       myEllipse class for defining a 3D Ellipse graphical object
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
        %   - The variables should match the name with the "ellipse" function attributes 
        hfunc                                                              % The function handle for this graphic object.                
        
        % x,y,z mesh data of the ellipse
        XData
        YData
        ZData
        

        
        % x,y,z center position data of the marker
                
        % Graphic attributes
        % Whole list of the graphic attributes, useful when we set the attributes and its values.
        gAttrList  = [ "FaceAlpha", "FaceColor", "LineStyle", "EdgeColor" ]; 
                   
        % "CData", ...
        % List of the graphic attributes that is updated for each time step.
        % Usually, the position datas should be updated
        gAttrUpdateList  = [ "XData", "YData", "ZData" ];                    
        
        FaceAlpha = 0.3; 
        FaceColor = 'flat';
        EdgeColor = [0, 0, 0];
        LineStyle = '-';
        

        
    end
    
    methods

        function obj = myEllipsoid( varargin )
            
            obj.type = "ellipsoid";
            obj.setAttr( varargin{ : } )                                   % The initialize function is defined under "myGraphics.m" file.
            
        end
        

        function h = create( obj, hplot )
            % Creating the primitive object,
            % This is only for 3D case!!! Hence no if-else statement.
  
            h = surf(  obj.XData( :, :, 1 ), obj.YData( :, :, 1 ), obj.ZData( :, :, 1 ), 'parent', hplot );                
                            
            % Setting the detailed graphical colors 
            for attr = obj.gAttrList 
                set( h, attr, obj.( attr ) );
            end
            
        end
        
        

    end
end


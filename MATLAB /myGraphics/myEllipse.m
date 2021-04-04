classdef myEllipse < handle
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
        % [1] Interal Function to check the size
        isSizeSame = @( x,y,z ) ( ( length( x ) == length( y ) ) && ...
                                    length( x ) == length( z ) );          

    end


    properties ( SetAccess = public )
        name
        xmesh 
        ymesh
        zmesh
        matrix
        center
        
        faceAlpha
        
        minAxes
        medAxes
        maxAxes
        singularValues
        
        % resolution, by default it is 30, meaning the mesh-grid is 30 by 30
    end
    
    methods

        function obj = myEllipse( matrix, center, varargin )
            % Construct an instance of the marker class
            % [Inputs Arguments]
            %      (1) matrix: A 3-by-3-by-N, 3 dimentional Matrix
            %         
            %      (2) center: A 3-by-N vector, contains the 3 dimensional position information 
            %       
            %  In total, let C = center and A = matrix, then (x-C)' A (x-C) <= 1
            obj.matrix = matrix;
            obj.center = center;
            
            r = myParser( varargin );
            
            for i = 1 : size( matrix, 3 ) 
                [ X,Y,Z ] = Ellipse_plot( obj.matrix(:,:,i), obj.center(:,i) );
                obj.xmesh( :, :, i ) = X;
                obj.ymesh( :, :, i ) = Y;
                obj.zmesh( :, :, i ) = Z;
                
                % Singular Value Decomposition of the Matrix
                [ U, S, ~ ] = svd( matrix(:,:,i) );
                
                obj.singularValues( :, :, i ) = S;

                % The matrix are in descending order, hence trimming the column vectors 
                obj.minAxes( :,i ) = U( :,1 ); 
                obj.medAxes( :,i ) = U( :,2 ); 
                obj.maxAxes( :,i ) = U( :,3 );                 
                
            end

            obj.name = "Ellipse" + num2str( randi ( 2^20 ) ) ;    
            obj.faceAlpha = r.faceAlpha;
                
        end
        

    end
end


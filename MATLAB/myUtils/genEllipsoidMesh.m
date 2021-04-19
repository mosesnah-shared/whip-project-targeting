function [ meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( varargin )
% Generate the ellipsoid mesh array, a "level-curve method", x^T (arrs) x <= 1
%
% =============================================================== %
% [INPUT] varagin: getting the whole list of inputs given
%        
% [PROPERTIES]                                                                                          
%     arrays: ( 3-by-3-by-N ) 3D matrices, which contains the Kx matrix. The matrix should be symmetric
%    centers: ( 3-by-N )      2D matrix, which contains the center's position of the matrices 
%      Nmesh: The resolution (or number of grids) of the meshgrid.                   
%       type: Type of the geometric shape that we are plotting. [#1] Is energy curve method, [#2] Is force ellipse method.
% =============================================================== %
%
% =============================================================== %
% [OUTPUT]    None
%   (1~3) mesh(X,Y,Z) ( Nmesh-by-Nmesh-by-N ) 3D matrices, which contains the surface mesh. For graphics.
%     [4] eigvals     ( Nmesh-by-Nmesh-by-N ) The eigenvalues of the matrices
%     [5] eigvecs     ( Nmesh-by-Nmesh-by-N ) The corresponding eigenvectors. 
%
% =============================================================== %
%
%
% =============================================================== %
%
% =============================================================== %
% SEE ALSO testHelpFunction 
%
% =============================================================== %
% 
% [CREATED BY]: Moses C. Nah
% [DATE]      : 18-April-2021
%
% =============================================================== %

% =============================================================== %
% [REFERENCES]
%
%
% =============================================================== %


    p = inputParser( );                                                    % Case sensitive is OFF By default
                                                                           % [REF] https://de.mathworks.com/help/matlab/ref/inputparser.html
    % [TIP] We can add a function handle for a simple input-type check
    ckc = @(x) ( isnumeric(x) && ( x > 0 ) );
                                                                           
    addParameter( p,      'arrays',  zeros( 3, 3, 1 ) );
    addParameter( p,     'centers',  zeros( 3, 3    ) );
    addParameter( p,       'Nmesh',          20, ckc );   
    addParameter( p,        'type',           1, ckc );                    % Type of the ellipse that we are plotting

    parse( p, varargin{ : } )

    r = p.Results;  
    M = r.Nmesh;

    
    N = size( r.arrays, 3 );                                               % The number of sample points. 

    eigvals = zeros(   3,   3, N );
    eigvecs = zeros(   3,   3, N );
    meshX   = zeros( M+1, M+1, N );
    meshY   = zeros( M+1, M+1, N );
    meshZ   = zeros( M+1, M+1, N );
 
    
    for i = 1 : N     % For 1-N; the length of the 3D matrices. 
        
        tmp = r.arrays( :, :, i );
        tmp = (tmp + tmp')/2;                                              % In case, trimming out the anti-symmetric part
        [ V, D ] = eig( tmp );                                             % Get the 3-by-3 matrices
        
        eigvals( :, :, i ) = D;
        eigvecs( :, :, i ) = V;                                            % Normalized eigenvectors. 
        
        if r.type == 1
        
            % Since the matrices are symmetric, D will be real. and 
            % x^T A x = x^T V D V^T x = y^T D y, where y = V^T x. 
            % Hence, it is literally finding y' = sqrt( D ) y <= 1

            % Getting the length of the major axes. 
            a = 1 / sqrt( D( 1,1 ) ) * 2;                              
            b = 1 / sqrt( D( 2,2 ) ) * 2;
            c = 1 / sqrt( D( 3,3 ) ) * 2;

        elseif r.type  == 2
                
            % Getting the length of the major axes. 
            
            % Find the maximum value;
            
            tmp = max( [ sqrt( D( 1,1 ) ), sqrt( D( 2,2 ) ), sqrt( D( 3,3 ) ) ]);
            
            a = sqrt( D( 1,1 ) ) / tmp * 0.05;                              
            b = sqrt( D( 2,2 ) ) / tmp * 0.05;
            c = sqrt( D( 3,3 ) ) / tmp * 0.05;                
                
        end
            
        
        [ X, Y, Z ] = ellipsoid( 0, 0, 0, a, b, c, M );                    % a,b,c are the leng
    
        
        XX = zeros( M+1, M+1 );
        YY = zeros( M+1, M+1 );
        ZZ = zeros( M+1, M+1 );        
        
        for j = 1 : M + 1
            for k = 1 : M + 1
                point = [ X( j,k ), Y( j,k ), Z( j,k ) ]';
                P = V * point;
                XX( j, k ) = P( 1 ) + r.centers( 1, i );
                YY( j, k ) = P( 2 ) + r.centers( 2, i );
                ZZ( j, k ) = P( 3 ) + r.centers( 3, i );
            end
        end        
            
        meshX( :, :, i ) = XX; 
        meshY( :, :, i ) = YY;
        meshZ( :, :, i ) = ZZ;

    end


end


function [ meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( arrs, centers, M )
% Generate the ellipsoid mesh array, a "level-curve method", x^T (arrs) x <= 1
%   input:  [1] arrs    ( 3-by-3-by-N ) 3D matrices, which contains the Kx matrix. The matrix should be symmetric
%           [2] centers ( 3-by-N )      2D matrices, which contains the center's position of the matrices 
%           [3] M       The resolution (or number of grids) of the meshgrid.                   
%   output: [1] mesh ( M-by-M-by-N ) 3D matrices, which contains the surface mesh. For graphics.
%           [2] eigvals - the eigenvalues of the matrices
%           [3] eigvecs - the corresponding eigenvectors. 

    N = size( arrs, 3 );                                                   % The number of sample points. 

    eigvals = zeros(   3,   3, N );
    eigvecs = zeros(   3,   3, N );
    meshX   = zeros( M+1, M+1, N );
    meshY   = zeros( M+1, M+1, N );
    meshZ   = zeros( M+1, M+1, N );
    
    for i = 1 : N     % For 1-N; the length of the 3D matrices. 
        
        tmp = arrs( :, :, i );
        tmp = (tmp + tmp')/2;                                              % In case, trimming out the anti-symmetric part
        [ V, D ] = eig( tmp );                                             % Get the 3-by-3 matrices
        
        eigvals( :, :, i ) = D;
        eigvecs( :, :, i ) = V;                                            % Normalized eigenvectors. 
        
        
        % Since the matrices are symmetric, D will be real. and 
        % x^T A x = x^T V D V^T x = y^T D y, where y = V^T x. 
        % Hence, it is literally finding y' = sqrt( D ) y <= 1
        
        % Getting the length of the major axes. 
        a = 1 / sqrt( D( 1,1 ) );                              
        b = 1 / sqrt( D( 2,2 ) );
        c = 1 / sqrt( D( 3,3 ) );
        
        
        [ X, Y, Z ] = ellipsoid( 0, 0, 0, a, b, c, M );                    % a,b,c are the leng
    
        
        XX = zeros( M+1, M+1 );
        YY = zeros( M+1, M+1 );
        ZZ = zeros( M+1, M+1 );        
        
        for j = 1 : M + 1
            for k = 1 : M + 1
                point = [ X( j,k ), Y( j,k ), Z( j,k ) ]';
                P = V * point;
                XX( j, k ) = P( 1 ) + centers( 1, i );
                YY( j, k ) = P( 2 ) + centers( 2, i );
                ZZ( j, k ) = P( 3 ) + centers( 3, i );
            end
        end        
            
        meshX( :, :, i ) = XX; 
        meshY( :, :, i ) = YY;
        meshZ( :, :, i ) = ZZ;

    end


end


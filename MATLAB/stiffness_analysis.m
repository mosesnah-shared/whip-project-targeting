% [Project]        [M3X] Whip Project
% [Title]          End-effector Cartesian Stiffness Analysis
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 12th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 
addpath( './myGraphics' ); addpath( './myUtils' ); addpath( './myRobots' );
myFigureConfig( 'fontsize', 20, ...
               'lineWidth', 5, ...
              'markerSize', 25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();                        

%% ==================================================================
%% (1-) Calculating the Manipulator equation and the Y matrix / a vector
%% -- (1A) Set the 2DOF Robot

% To use the 2DOF robot, use the following line
% robot = my2DOFRobot( );     

% To use the 4DOF robot, use the following line
robot = my4DOFRobot( );     

robot.initialize( );
[M, C, G] = robot.deriveManipulatorEquation( );
J         = robot.getEndEffectorJacobian(  );


%% -- (1B) The symbolic form of the end-effector stiffness (compliance) matrix and its function

Kq = [17.4,  6.85, -7.75, 8.40;
       6.85, 33.0,  3.70, 0.00;
      -7.75, 3.70, 27.70, 0.00;
       8.40, 0.00, 0.00, 23.2];
Cq = Kq^-1; 


Cx = J * Cq * J.';

sym_array = [ robot.L ];
val_array = { 0.294, 0.291 };  % Actual Length of robot
pEL = robot.forwardKinematics( 2, [ 0; 0;             0 ] );               % Position of the elbow
pEE = robot.forwardKinematics( 2, [ 0; 0; -robot.L( 2 ) ] );               % Position of the end-effector      


% Substituting the symbol's values to values
J   = subs(   J, sym_array, val_array );                                   % Jacobian of the end-effector
Cx  = subs(  Cx, sym_array, val_array );                                   % End-effector Cartesian Compliance Matrix
pEL = subs( pEL, sym_array, val_array );                                   % Position of the elbow
pEE = subs( pEE, sym_array, val_array );                                   % Position of the end-effector      

% Making the symbolic expression to a matlabFunction. This increases the speed tremendously.
tmp1 = arrayfun( @char,   J, 'uniform', 0 );
tmp2 = arrayfun( @char,  Cx, 'uniform', 0 );
tmp3 = arrayfun( @char, pEL, 'uniform', 0 );
tmp4 = arrayfun( @char, pEE, 'uniform', 0 );

tmp1 = replace( tmp1, ["q1(t)", "q2(t)", "q3(t)", "q4(t)"], ["q1", "q2", "q3", "q4" ] );
tmp2 = replace( tmp2, ["q1(t)", "q2(t)", "q3(t)", "q4(t)"], ["q1", "q2", "q3", "q4" ] );
tmp3 = replace( tmp3, ["q1(t)", "q2(t)", "q3(t)", "q4(t)"], ["q1", "q2", "q3", "q4" ] );
tmp4 = replace( tmp4, ["q1(t)", "q2(t)", "q3(t)", "q4(t)"], ["q1", "q2", "q3", "q4" ] );

J_func   = matlabFunction( str2sym( tmp1 ) );
Cx_func  = matlabFunction( str2sym( tmp2 ) );
pEL_func = matlabFunction( str2sym( tmp3 ) );
pEE_func = matlabFunction( str2sym( tmp4 ) );




%% -- (1C) Graph of Cartesian Stiffness ellipses

clear gObjs

% Types of q pos values that you'll test. 

% qMat = [  pi/2,  pi/2, pi/2, 0.05;
%           pi/2,  pi/4, pi/2, 0.05;
%           pi/2,     0, pi/2, 0.05;
%           pi/2, -pi/4, pi/2, 0.05;]

nR = size( qMat, 1 );     

gObjs( 1 ) = myMarker( 'XData', 0, 'YData', 0 , 'ZData', 0   , ... 
                        'name', "SH" , 'SizeData',  250 , ...
                   'LineWidth',   3                          , ...
             'MarkerEdgeColor',  c.green                     , ...
             'MarkerFaceColor',  c.white                     , ...
             'MarkerFaceAlpha', 0.8                       );               % The starting point, the shoulder marker.

for i = 1 : nR
       
    pEL_pos = pEL_func( qMat( i, 1 ), qMat( i, 2 )                             );
    pEE_pos = pEE_func( qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );    
    

    gObjs( 2 * i    ) = myMarker( 'XData', pEL_pos( 1 ), 'YData', pEL_pos( 2 ) , 'ZData', pEL_pos( 3 ), ... 
                            'name', strcat( "EL_", num2str( i ) ) , 'SizeData',  250   , ...
                       'LineWidth',   3                       , ...
                 'MarkerEdgeColor',  c.orange                 , ...
                 'MarkerFaceColor',  c.white                  , ...
                 'MarkerFaceAlpha', 0.8                       );          % Defining the markers for the plot


    gObjs( 2 * i + 1 ) = myMarker( 'XData', pEE_pos( 1 ), 'YData',  pEE_pos( 2 ) , 'ZData',  pEE_pos( 3 ), ... 
                            'name', strcat( "EE_", num2str( i ) ) , 'SizeData',  250   , ...
                       'LineWidth',   3                       , ...
                 'MarkerEdgeColor',  c.blue                   , ...
                 'MarkerFaceColor',  c.white                  , ...
                 'MarkerFaceAlpha', 0.8                       );          % Defining the markers for the plot


             
end

ani = myAnimation( 0.1, gObjs );                         % Input (1) Time step of sim. 

for i = 1 : nR
    ani.connectMarkers( 1, [ "SH", strcat( "EL_", num2str( i ) ), strcat( "EE_", num2str( i ) ) ], ...
                                        'Color', c.grey, 'LineStyle',  '-' );            
    
    pEE_pos = pEE_func( qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );    
    
    % Superimposing the stiffness matrix
    Cx = Cx_func( qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx = Cx^-1;    
    
    [meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( Kx, pEE_pos, 20, 1 );
    

    gObjs( end + 1 ) = myEllipsoid( 'XData', meshX, 'YData',  meshY , 'ZData',  meshZ  );  
    ani.addGraphicObject( 1, gObjs( end ) );
    
end


tmpLim = 0.4;
set( ani.hAxes{ 1 }, 'XLim', [  0.15 - tmpLim,  0.15 + tmpLim ] , ...                  
                     'YLim', [ -0.30 - tmpLim, -0.30 + tmpLim ] , ...    
                     'ZLim', [  0.00 - tmpLim,  0.00 + tmpLim ] , ...
                     'view', [41.8506   15.1025 ]     )  
set( ani.hAxes{ 1 }, 'LineWidth', 1.4 )
                 
view(0,90)  % XY
camroll( 90 )
xlabel( "x [m]" ); ylabel( "y [m]" ); zlabel( "z [m]" )
% ani.run( 0.1, 0.1, true, ['output', num2str( idx ) ])


%% -- (2A) Real time animation of the optimal movement

idx  = 3;
data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );


clear gObjs

% Marker in order, upper limb 3 joints( SH, EL, WR) 
genNodes = @(x) ( "node" + (1:x) );
N        = 25;

tmpC = [ c.pink; c.blue; c.green ];   

% For the target of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "target" , ...
                     'SizeData',  250            , ...
                    'LineWidth',   3             , ...
              'MarkerEdgeColor',  tmpC( idx, : ) , ...
              'MarkerFaceColor',  tmpC( idx, : ) , ...
              'MarkerFaceAlpha', 0.8            );                         % Defining the markers for the plot

          

stringList = [      "SH", "EL", "EE",     genNodes( N ) ];                                
sizeList   = [           400,      400,      400, 75 * ones( 1, N ) ];                            
colorList  = [ repmat( tmpC( idx, : ), 3, 1 ); repmat( [0.5, 0.5, 0.5], N , 1 ) ];                            
           

% For the whole model
for i = 1 : length( stringList )
    
    
    
   gObjs( i + 1 ) = myMarker( 'XData', data.geomXYZPositions( 3 * i + 1, :  ) , ... 
                              'YData', data.geomXYZPositions( 3 * i + 2, :  ) , ... 
                              'ZData', data.geomXYZPositions( 3 * i + 3, :  ) , ... 
                               'name', stringList( i ), ...
                           'SizeData',   sizeList( i ), ...
                          'LineWidth',   7            , ...
                    'MarkerEdgeColor',  colorList( i, : ) ); 

end

          
          
% For the ZFT Postures
% Forward Kinematics, the ZFT Positions

pSH_ZFT  = zeros( 3, length( data.currentTime ) ); 
pEL_ZFT  = reshape( pEL_func( data.pZFT(1,:)', data.pZFT(2,:)'                                  ), [], 3 )';
pEE_ZFT  = reshape( pEE_func( data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)'), [], 3 )';

pos = { pSH_ZFT, pEL_ZFT, pEE_ZFT };
 

stringList = [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ];                                
sizeList   = [      400,      400,      400 ];                            
colorList  = repmat( tmpC( idx, : ), 3, 1 );           

for i = 1 : length( pos )
   gObjs( end + 1 ) = myMarker( 'XData', pos{ i }( 1, :  ) , ... 
                                'YData', pos{ i }( 2, :  ) , ... 
                                'ZData', pos{ i }( 3, :  ) , ... 
                                 'name', stringList( i ), ...
                             'SizeData',   sizeList( i ) * 1.0, ...
                            'LineWidth',   7            , ...
                      'MarkerEdgeColor',  colorList( i, : ), ...
                      'MarkerFaceAlpha', 0.3 , ...
                      'MarkerEdgeAlpha', 0.3 ); 

end


% Preparing the stiffness matrix for the plot. 

qMat = data.jointAngleActual( 1:4, : )';

for i = 1 : length( data.currentTime)           % Calculation for each 
            
    % Superimposing the stiffness matrix
    Cx = Cx_func( qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx = Cx^-1;    

    Kx_whole( :, :, i ) = Kx;

end

[meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( Kx_whole, data.geomXYZPositions( 10:12, : ), 20, 1 );

gObjs( end + 1 ) = myEllipsoid( 'XData', meshX, 'YData',  meshY , 'ZData',  meshZ );  


ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.grey, 'LineStyle',  '-' );      
ani.connectMarkers( 1, [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ], 'Color', c.grey, 'LineStyle', '--' );      

tmpC = [ c.pink; c.green; c.blue; c.yellow ];

% Add the 2nd figure plot
ani.adjustFigures( 2 );                     

for i = 1 : 4
    plot( ani.hAxes{ 2 }, data.currentTime, data.pZFT( i, : ), 'color', tmpC( i, : ) , 'linestyle', '--','linewidth', 5 );
    tmp = myMarker( 'XData', data.currentTime , 'YData', data.jointAngleActual( i, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  tmpC( i, : )  ); 
    ani.addTrackingPlots( 2, tmp );
    
end

tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [41.8506   15.1025 ]     )           
%                      'view',   [41.8506   15.1025 ]     )                


ani.addZoomWindow( 3 , "EE", 0.6 );   
set( ani.hAxes{ 3 },  'view',   [41.8506   15.1025 ]     ) 

set( ani.hAxes{ 1 }, 'LineWidth', 1.4 ); set( ani.hAxes{ 3 }, 'LineWidth', 1.4 );

tmp1 = 40;

xlabel( ani.hAxes{ 1 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 1 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 1 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 3 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 3 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 3 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 2 }, 'Time [sec]', 'Fontsize', 30 ); ylabel( ani.hAxes{ 2 }, '$q, q_0$ [rad]', 'Fontsize', tmp1);
set( ani.hAxes{ 2 }, 'LineWidth', 1.4, 'XLim', [0, 3] )

ani.run( 0.33, 2.3, true, ['output', num2str( idx ) ])

%% -- (3A) Analysis of velocity vector vs. eigenvectors

% Choose the set of data

idx  = 2;
D = [0.950, 0.579, 0.950];

data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );


qMat = data.jointAngleActual( 1:4, : )';

for i = 1 : length( data.currentTime)           % Calculation for each 
            
    % Superimposing the stiffness matrix
    Cx = Cx_func( qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx = Cx^-1;    

    Kx_whole( :, :, i ) = Kx;

end

[meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( Kx_whole, zeros(3, length( data.currentTime ) ), 20, 1 );

EEvel  = data.geomXYZVelocities( 10:12, :  );

nEEvel = normc( EEvel );
% Trim out the eigenvectors with the corresponding eigenvalues. 

for i = 1 : size( eigvals, 3 ) 
   
    tmp    = diag( eigvals( :, :, i ) );
    tmpIdx = find( tmp == min( tmp ) );
    
    minEigVecs( :, i ) = eigvecs( :, tmpIdx, i );
    
    dotVal( i ) = dot( nEEvel( :, i ), minEigVecs( :, i ) );
end

clear gObjs

gObjs( 1 ) = myEllipsoid( 'XData', meshX, 'YData',  meshY , 'ZData',  meshZ );  


tmpZeros = zeros( 1, length( data.currentTime) );
scale = 4;
EEvel = EEvel/scale;
gObjs( 2 ) = myArrow( 'XData', tmpZeros, 'YData', tmpZeros, 'ZData', tmpZeros, ...
                      'UData', EEvel( 1, : ), 'VData', EEvel( 2, : ), 'WData', EEvel( 3, : ), 'LineWidth', 3 );
                   
% 
% ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
% 
% set( ani.hAxes{ 1 }, 'LineWidth', 1.4 ); 
% 
% tmpLim = 0.7;
% set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
%                      'YLim',   [ -tmpLim , tmpLim ] , ...    
%                      'ZLim',   [ -tmpLim , tmpLim ] , ...
%                      'view',   [41.8506   15.1025 ]     )  
%                  
%  ani.run( 0.33, 2.0, true, ['output', num2str( idx ) ])

plot( data.currentTime, abs( dotVal ) )
 
tmpIdx = find( data.outputVal == min( data.outputVal ) );
hold on

set( gca, 'xlim', [0.10, data.currentTime( tmpIdx ) ], 'LineWidth', 1.4 )
h = fill([0, 0, 0.1+D(idx), 0.1+D(idx) ],[0 1 1 0],'k')
set(h,'facealpha',.1 , 'edgealpha',0 )

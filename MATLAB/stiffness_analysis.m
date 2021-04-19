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
myFigureConfig( 'fontsize',  20, ...
               'LineWidth',  10, ...
           'AxesLineWidth', 1.5, ...     For Grid line, axes line width etc.
              'markerSize',  25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();                        

%% ==================================================================
%% (1-) Calculating and Plotting the stiffness matrix 
%% -- (1A) Set the 2DOF Robot

% To use the 2DOF robot, use the following line
% robot = my2DOFRobot( );     

% To use the 4DOF robot, use the following line
robot = my4DOFRobot( );     

robot.initialize( );
% [M, C, G] = robot.deriveManipulatorEquation( );                          % You don't need dynamic equation for this part so commenting out.
J         = robot.getEndEffectorJacobian(  );


%% -- (1B) The symbolic form of the end-effector stiffness (compliance) matrix and its function

Kq = [17.4,  6.85, -7.75, 8.40;
       6.85, 33.0,  3.70, 0.00;
      -7.75, 3.70, 27.70, 0.00;
       8.40, 0.00, 0.00, 23.2];
% 
% Bq = 0.05 * Kq;   
   
% Kq = 30 * eye( 4 );

Cq = Kq^-1; 
Cx = J * Cq * J.';

L1 = 0.294; L2 = 0.291; 

pEL = robot.forwardKinematics( 2, [ 0; 0;             0 ] );               % Position of the elbow
pEE = robot.forwardKinematics( 2, [ 0; 0; -robot.L( 2 ) ] );               % Position of the end-effector      

old = [ "q1(t)", "q2(t)", "q3(t)", "q4(t)" ]; new = [ "q1", "q2", "q3", "q4" ];

J_func   = myFunctionize(    J, old, new );
Cx_func  = myFunctionize(   Cx, old, new );
pEL_func = myFunctionize(  pEL, old, new );
pEE_func = myFunctionize(  pEE, old, new );


%% -- (1C) Graph of Cartesian Stiffness ellipses

clear gObjs;
% Types of q pos values that you'll test. 
% qMat = [ 11 * pi/12, 5 * pi/6, pi/3, 0.8];


qMat = [  1.72788, 0.     , 0.     , 0.33161;
          1.72788,-1.03427,-1.39626, 0.19199;
          1.72788,+1.03427,-1.39626, 0.19199;
         2.67035,-0.69813,-1.39626, 0.05236 ];

% [2021.04.19] [MOSES NAH] Backup!! 
% qMat = [  pi/2,  pi/2, pi/2, 0.05;
%           pi/2,  pi/4, pi/2, 0.05;
%           pi/2,     0, pi/2, 0.05;
%           pi/2, -pi/4, pi/2, 0.05;];

nR = size( qMat, 1 );     

% Drawing the shoulder marker first, which is the origin of the Cartesian Coordinate.
gObjs( 1 ) = myMarker( 'XData', 0, 'YData', 0 , 'ZData', 0, 'name', "SH" , 'SizeData',  250 , ...
                   'LineWidth',   3                          , ...
             'MarkerEdgeColor',  c.green                     , ...
             'MarkerFaceColor',  c.white                     , ...
             'MarkerFaceAlpha', 0.8                       );         
         
         
% Iterating through the joint values and plotting the elbow, end-effector markers. 
for i = 1 : nR
       
    pEL_pos = pEL_func( L1,     qMat( i, 1 ), qMat( i, 2 )                             );
    pEE_pos = pEE_func( L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );    
    
    % ELBOW MARKER
    gObjs( end + 1 ) = myMarker( 'XData', pEL_pos( 1 ), 'YData', pEL_pos( 2 ) , 'ZData', pEL_pos( 3 ), ... 
                            'name', strcat( "EL_", num2str( i ) ) , 'SizeData',  250   , ...
                       'LineWidth',   3                       , ...
                 'MarkerEdgeColor',  c.orange                 , ...
                 'MarkerFaceColor',  c.white                  , ...
                 'MarkerFaceAlpha', 0.8                       );            

    % SHOULDER MARKER
    gObjs( end + 1 ) = myMarker( 'XData', pEE_pos( 1 ), 'YData',  pEE_pos( 2 ) , 'ZData',  pEE_pos( 3 ), ... 
                            'name', strcat( "EE_", num2str( i ) ) , 'SizeData',  250   , ...
                       'LineWidth',   3                       , ...
                 'MarkerEdgeColor',  c.blue                   , ...
                 'MarkerFaceColor',  c.white                  , ...
                 'MarkerFaceAlpha', 0.8                       );       
             
    % Superimposing the stiffness matrix ellipse representation.
    pEE_pos = pEE_func(  L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );    
    
    Cx = Cx_func(  L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx = Cx^-1;    
    
    % Generating the mesh array for 
    [meshX, meshY, meshZ, ~, ~] = genEllipsoidMesh( 'arrays', Kx, 'centers', pEE_pos, 'Nmesh', 40, 'Type', 1 );
    
    % Append the mesh arrays for plot.
    gObjs( end + 1 ) = myEllipsoid( 'XData', meshX, 'YData',  meshY , 'ZData',  meshZ  );               
             
end

ani = myAnimation( 0.1, gObjs );                       

for i = 1 : nR
    ani.connectMarkers( 1, [ "SH", strcat( "EL_", num2str( i ) ), strcat( "EE_", num2str( i ) ) ], ...
                                        'Color', c.grey, 'LineStyle',  '-' );            
    
end



tmpLim = 1;
set( ani.hAxes{ 1 }, 'XLim', [  0.00 - tmpLim,  0.00 + tmpLim ] , ...                  
                     'YLim', [  0.00 - tmpLim,  0.00 + tmpLim ] , ...    
                     'ZLim', [  0.00 - tmpLim,  0.00 + tmpLim ] , ...
                     'view', [41.8506   15.1025 ]     )  
                 
% view( 0,90 ); camroll( 90 );                                             % In case if you want to see the XY plane.
xlabel( "X [m]" ); ylabel( "Y [m]" ); zlabel( "Z [m]" )

%% ==================================================================
%% (2-) Animation
%% -- (2A) Real time animation of the optimal movement + Cartesian Stiffness Matrix

clear gObjs

% idx of the data that we are plotting, 1~3 correspond to target number.
idx  = 1; 
% data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );

% [2021.04.19] [MOSES NAH] Backup!! 
% This is for special case when target is closer.
data = myTxtParse( "./myData/data_log_T1_bad.txt" );

% The duration of ZFT of the optimal upper limb movement. 
D = [0.950, 0.579, 0.950];

% Marker in order, upper limb 3 joints( SH, EL, WR) 
genNodes = @(x) ( "node" + (1:x) );
N        = 25;

tmpC = [ c.pink; c.blue; c.green ];   

% For the target of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "target" , 'SizeData',  250    , ...
                    'LineWidth',   3             , ...
              'MarkerEdgeColor',  tmpC( idx, : ) , ...
              'MarkerFaceColor',  tmpC( idx, : ) , ...
              'MarkerFaceAlpha', 0.8            );                         % Defining the markers for the plot


stringList = [          "SH",     "EL",     "EE",     genNodes( N ) ];                                
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
% Forward Kinematics, the ZFT Positions of the SH, EL and EE.
pSH_ZFT  = zeros( 3, length( data.currentTime ) ); 
pEL_ZFT  = reshape( pEL_func( L1, data.pZFT(1,:)', data.pZFT(2,:)'                                  ), [], 3 )';
pEE_ZFT  = reshape( pEE_func( L1, L2, data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)' ) , [], 3 )';


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

for i = 1 : length( data.currentTime ) 
            
    % Superimposing the stiffness matrix
    Cx = Cx_func( L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx_whole( :, :, i ) = Cx^-1;
    
end


[meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( 'arrays', Kx_whole, 'centers', data.geomXYZPositions( 10:12, : ), 'Nmesh', 40, 'Type', 1 );

% Finding the dot product between the end-effector velocity and 
% First, normalizing the end-effector velocity.

EEvel  = data.geomXYZVelocities( 10:12, :  );
nEEvel = normc( EEvel );
for i = 1 : size( eigvals, 3 )  
   
    tmp    = diag( eigvals( :, :, i ) );
%     tmpIdx = find( tmp == max( tmp ) );         % For maximum value.
    tmpIdx = find( tmp == min( tmp ) );         % For minimum value
    
    myEigVecs( :, i ) = eigvecs( :, tmpIdx, i );
    
    dotVal( i ) = dot( nEEvel( :, i ), myEigVecs( :, i ) );
    
    dotVal2( i ) = dot( EEvel( :, i ), myEigVecs(:,i ) );
end

gObjs( end + 1 ) = myEllipsoid( 'XData', meshX, 'YData',  meshY , 'ZData',  meshZ );  

% scale = 4;
% gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData',  data.geomXYZPositions( 12, : ), ...
%                             'UData', data.forceVec( 1, : )/scale,    'VData', data.forceVec( 2, : )/scale,    'WData', data.forceVec( 3, : )/scale, 'LineWidth', 10, 'Color', c.pink );
                        
ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array
ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.grey, 'LineStyle',  '-' );      
ani.connectMarkers( 1, [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ], 'Color', c.grey, 'LineStyle', '--' );      


% Colors for J1, J2, J3, J4
tmpC = [ c.pink; c.green; c.blue; c.yellow ];

% Add the 2nd figure plot
ani.adjustFigures( 2 );                     
% for i = 1 : 4
%     plot( ani.hAxes{ 2 }, data.currentTime, data.pZFT( i, : ), 'color', tmpC( i, : ) , 'linestyle', '--','linewidth', 5 );
%     tmp = myMarker( 'XData', data.currentTime , 'YData', data.jointAngleActual( i, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
%                  'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  tmpC( i, : )  ); 
%     ani.addTrackingPlots( 2, tmp );
%     
% end

% [BACKUP] When we want to plot the time vs dot value .
% plot( ani.hAxes{ 2 }, data.currentTime, dotVal, 'color', c.blue, 'linestyle', '--','linewidth', 5 );
tmp = myMarker( 'XData', data.currentTime , 'YData', abs( dotVal  ), 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  c.blue  ); 

ani.addTrackingPlots( 2, tmp );

tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [41.8506   15.1025 ]     )           
ani.addZoomWindow( 3 , "EE", 0.6 );   

set( ani.hAxes{ 3 },  'view',   [41.8506   15.1025 ]     ) 


tmp1 = 40;
xlabel( ani.hAxes{ 1 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 1 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 1 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 2 }, 'Time [sec]', 'Fontsize', 30 ); 

ani.run( 0.33, 2.9, true, ['output', num2str( idx ) ])

%% ==================================================================
%% (3-) Static Plots
%% -- (3A) Static Plots of Force, end-effector trajectories and etc.

idx  = 3;
data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );

ttmp = 50;

EEpos = data.geomXYZPositions(  10:12, 1:ttmp );
EEvel = data.geomXYZVelocities( 10:12, 1:ttmp ); 

plot3( EEpos( 1,: ), EEpos( 2,: ), EEpos( 3,: ), 'color', c.pink, 'linewidth', 5 );
hold on 
scatter3( EEpos( 1,: ), EEpos( 2,: ), EEpos( 3,: ), 400, 'o', 'markeredgecolor', c.pink, 'markerfacecolor', c.white, 'markerfacealpha', 0.8, 'linewidth', 6 );

% Adding force vector.
scale = 0.01;
quiver3(  data.geomXYZPositions( 10,: ), data.geomXYZPositions( 11, : ), data.geomXYZPositions( 12,: ),...
          data.forceVec( 1, :) * scale, data.forceVec( 2, :) * scale, data.forceVec( 3, :) * scale, ...
          'linewidth', 5, 'color', c.blue, 'AutoScale', 'off'  )

axis equal

qMat = data.jointAngleActual( 1:4, 1:ttmp )';
for i = 1: ttmp
   J_vals( :, :, i ) = J_func( L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
   ZFTvel( :, i ) = J_vals( :, :, i ) * data.jointVelActual( 1:4, i );
   
   Cx = Cx_func(  L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
   Kx = Cx^-1;       
   
   tmp1( :, i ) = Kx * ( pEE_ZFT( :, i ) - EEpos( :, i ) );
end

% What is the force due to Kx x

% tmp2 = 0.05 * Kx * ( ZFTvel - EEvel );

% tmp = tmp1 + tmp2;
tmp = tmp1;

% scale = 0.0000001;
quiver3(  EEpos( 1,: ), EEpos( 2, : ), EEpos( 3,: ),...
          tmp( 1, :) * scale, tmp( 2, :) * scale, tmp( 3, :) * scale, ...
          'linewidth', 5, 'color', c.green, 'AutoScale', 'on'  )

%% ==================================================================
%% (4-) Analysis
%% -- (4A) Analysis of velocity vector vs. eigenvectors

% data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );
data = myTxtParse( "./myData/data_log_tmp.txt" );
qMat = data.jointAngleActual( 1:4, : )';
for i = 1 : length( data.currentTime)           % Calculation for each 
            
    % Superimposing the stiffness matrix
    Cx = Cx_func( L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
    Kx = Cx^-1;    
    Kx_whole( :, :, i ) = Kx;
end


[meshX, meshY, meshZ, eigvals, eigvecs] = genEllipsoidMesh( 'arrays', Kx_whole, 'centers',zeros(3, length( data.currentTime ) ),'Nmesh', 40, 'Type', 1 );
EEvel  = data.geomXYZVelocities( 10:12, :  );
nEEvel = normc( EEvel );
% Trim out the eigenvectors with the corresponding eigenvalues. 


% Choose the set of data
idx  = 2;
D = [0.950, 0.579, 0.950];

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


%% -- (4B) Force vs. Displacement
N = length( data.currentTime );
clear gObjs
pEE_ZFT  = reshape( pEE_func( data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)'), [], 3 )';
pEE      = data.geomXYZPositions( 10:12 , : );
dx0       = [zeros(3,1), diff( pEE, 1, 2)];
% dx0 = pEE_ZFT - pEE;
dx0  = normc( dx0 );
fVec = normc( data.forceVec );
scale = 4;
gObjs( 1 ) = myArrow( 'XData', zeros( 1, N ), 'YData',  zeros( 1, N ), 'ZData',  zeros( 1, N ), ...
                      'UData', fVec( 1, : ), 'VData', fVec( 2, : ), 'WData', fVec( 3, : ), 'LineWidth', 10, 'Color', c.pink );
                  
gObjs( 2 ) = myArrow( 'XData', zeros( 1, N ), 'YData',  zeros( 1, N ), 'ZData',  zeros( 1, N ), ...
                      'UData', dx0( 1, : ), 'VData', dx0( 2, : ), 'WData', dx0(3, : ), 'LineWidth', 10, 'Color', c.blue );                  


ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
dot = sum( fVec.* dx0 );
tmpLim = 1;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [41.8506   15.1025 ]     )          
                 
ani.run( 0.33, 3.0, false, '1')    


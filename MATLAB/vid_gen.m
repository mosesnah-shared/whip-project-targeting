% [Project]        [M3X] Whip Project
% [Title]          Script for generating the images 
% [Author]         Moses C. Nah
% [Creation Date]  Monday, June 4th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu
%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 
addpath( './myGraphics' ); addpath( './myUtils' ); addpath( './myRobots' );
myFigureConfig( 'fontsize', 40, ...
               'lineWidth', 10, ...
              'markerSize', 25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();            


%% ==================================================================
%% (1-) Video Geneartion
%% -- (1A) Calling the data for animation

idx  = 3;
data = myTxtParse( ['./myData/simulation_log/data_log_T', num2str( idx ), '.txt' ] );

%% -- (1B) For the forward kinematics

% To use the 4DOF robot, use the following line
robot = my4DOFRobot( );     

robot.initialize( );
[M, C, G] = robot.deriveManipulatorEquation( );
J         = robot.getEndEffectorJacobian(  );

sym_array = [ robot.M, robot.L, robot.Lc, reshape( robot.I', 1, [] ), robot.g ];
val_array = { 1.595, 0.869, ... Mass   of each limb segment, ordered from proximal to distal (upperarm - forearm)
              0.294, 0.291, ... Length of each limb segment, ordered from proximal to distal (upperarm - forearm)
              0.129, 0.112, ... Length from proximal joint to center of mass, ordered from proximal to distal (upperarm - forearm)
             0.0119, 0.0119, 0.0013, ... Moment of inertia of I1xx, I1yy, I1zz, w.r.t. center of mass.
             0.0048, 0.0049, 0.0005, ... Moment of inertia of I2xx, I2yy, I2zz, w.r.t. center of mass.
                  9.81 };  % Gravity

              
M_val = subs( M, sym_array, val_array );
C_val = subs( C, sym_array, val_array );
G_val = subs( G, sym_array, val_array );

% For animation, the forward kinematics of the elbow and end-effector joint
pEL = robot.forwardKinematics( 2, [ 0; 0;             0 ] );               % Position of the elbow
pEE = robot.forwardKinematics( 2, [ 0; 0; -robot.L( 2 ) ] );               % Position of the end-effector

old = [ "q1(t)", "q2(t)", "q3(t)", "q4(t)" ]; new = [ "q1", "q2", "q3", "q4" ];

pEL_func = myFunctionize( pEL, old, new );
pEE_func = myFunctionize( pEE, old, new );



%% -- (1B) Plot of Movement Snapshots

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
colorList  = [ repmat( c.orange_milky, 3, 1 ); repmat( c.purple, N , 1 ) ];                            
           

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
pEL_ZFT  = reshape( pEL_func( 0.294, data.pZFT(1,:)', data.pZFT(2,:)'                                  ), [], 3 )';
pEE_ZFT  = reshape( pEE_func( 0.294, 0.291, data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)'), [], 3 )';

pos = { pSH_ZFT, pEL_ZFT, pEE_ZFT };
 

stringList = [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ];                                
sizeList   = [      400,      400,      400 ];                            
colorList  = repmat( tmpC( idx, : ), 3, 1 );           
% 
% for i = 1 : length( pos )
%    gObjs( end + 1 ) = myMarker( 'XData', pos{ i }( 1, :  ) , ... 
%                                 'YData', pos{ i }( 2, :  ) , ... 
%                                 'ZData', pos{ i }( 3, :  ) , ... 
%                                  'name', stringList( i ), ...
%                              'SizeData',   sizeList( i ) * 1.0, ...
%                             'LineWidth',   7            , ...
%                       'MarkerEdgeColor',  colorList( i, : ), ...
%                       'MarkerFaceAlpha', 0.3 , ...
%                       'MarkerEdgeAlpha', 0.3 ); 
% 
% end




ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.grey, 'LineStyle',  '-' );      
% ani.connectMarkers( 1, [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ], 'Color', c.grey, 'LineStyle', '--' );      

tmpC = [ c.pink; c.green; c.blue; c.yellow ];




% Add the 2nd figure plot
% ani.adjustFigures( 2 );                     
% 
% for i = 1 : 4
%     plot( ani.hAxes{ 2 }, data.currentTime, data.pZFT( i, : ), 'color', tmpC( i, : ) , 'linestyle', '--','linewidth', 5 );
%     tmp = myMarker( 'XData', data.currentTime , 'YData', data.jointAngleActual( i, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
%                  'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  tmpC( i, : )  ); 
%     ani.addTrackingPlots( 2, tmp );
%     
% end

tmpLim = 2.5;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [41.8506   15.1025 ]     )           
%                      'view',   [41.8506   15.1025 ]     )                


% ani.addZoomWindow( 3 , "EE", 0.6 );   
% set( ani.hAxes{ 3 },  'view',   [41.8506   15.1025 ]     ) 

set( ani.hAxes{ 1 }, 'LineWidth', 1.4 )
set( ani.hAxes{ 3 }, 'LineWidth', 1.4 )


tmp1 = 40;

set( ani.hAxes{1}, 'xtick', [-2, 0, 2] ); set( ani.hAxes{1}, 'xticklabel', ["-2", "X[m]", "+2"], 'xticklabelrotation', 0 ) % ["-2", "X[m]", "+2"] )
set( ani.hAxes{1}, 'ytick', [-2, 0, 2] ); set( ani.hAxes{1}, 'yticklabel', ["-2", "Y[m]", "+2"], 'yticklabelrotation', 0 ) % ["-2", "Y[m]", "+2"] )
set( ani.hAxes{1}, 'ztick', [-2, 0, 2] ); set( ani.hAxes{1}, 'zticklabel', ["-2", "Z[m]", "+2"], 'zticklabelrotation', 0 ) % ["-2", "Z[m]", "+2"] )
set( ani.hAxes{1},'LineWidth',3.5 ); set(ani.hAxes{1}, 'TickLength',[0.04 0.04]);

ani.run( 0.5, 3.0, true, ['output', num2str( idx ) ])

%% ==================================================================
%% (2-) [2021 DOSIM WORKSHOP] Video Geneartion
%% -- (2A) Reading the Datas

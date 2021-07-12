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

idx  = 5;
% data = myTxtParse( ['./myData/simulation_log/data_log_T', num2str( idx ), '.txt' ] );
data = myTxtParse( ['./myData/for_do_sim/data_log_T', num2str( idx ), '.txt' ] );

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



% For the target of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "target"  , ...
                     'SizeData',  500      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   [1,0,0] , ...
              'MarkerFaceColor',   [1,0,0] , ...
              'MarkerFaceAlpha', 0.8       );                         % Defining the markers for the plot

          

stringList = [      "SH", "EL", "EE",     genNodes( N ) ];                                
sizeList   = [           700,      700,      700, 200 * ones( 1, N ) ];                            
colorList  = [ repmat( c.black, 3, 1 ); repmat( [0, 0.4470, 0.7410], N , 1 ) ];                            
           
% For the whole model
for i = 1 : length( stringList )
   gObjs( i + 1 ) = myMarker( 'XData', data.geomXYZPositions( 3 * i + 1, :  ) , ... 
                              'YData', data.geomXYZPositions( 3 * i + 2, :  ) , ... 
                              'ZData', data.geomXYZPositions( 3 * i + 3, :  ) , ... 
                               'name', stringList( i ), ...
                           'SizeData',   sizeList( i ), ...
                          'LineWidth',   15            , ...
                    'MarkerEdgeColor',  colorList( i, : ) ); 

end

          
          
% For the ZFT Postures
% Forward Kinematics, the ZFT Positions

% pSH_ZFT  = zeros( 3, length( data.currentTime ) ); 
% pEL_ZFT  = reshape( pEL_func( 0.294, data.pZFT(1,:)', data.pZFT(2,:)'                                  ), [], 3 )';
% pEE_ZFT  = reshape( pEE_func( 0.294, 0.291, data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)'), [], 3 )';
% 
% pos = { pSH_ZFT, pEL_ZFT, pEE_ZFT };
 

stringList = [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ];                                
sizeList   = [      400,      400,      400 ];                            
% colorList  = repmat( tmpC( idx, : ), 3, 1 );           
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

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' );      
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

set( ani.hAxes{1}, 'xtick', [-2, 0, 2] ); set( ani.hAxes{1}, 'xticklabel', ["-2", "X (m)", "+2"], 'xticklabelrotation', 0 ) % ["-2", "X[m]", "+2"] )
set( ani.hAxes{1}, 'ytick', [-2, 0, 2] ); set( ani.hAxes{1}, 'yticklabel', ["-2", "Y (m)", "+2"], 'yticklabelrotation', 0 ) % ["-2", "Y[m]", "+2"] )
set( ani.hAxes{1}, 'ztick', [-2, 0, 2] ); set( ani.hAxes{1}, 'zticklabel', ["-2", "Z (m)", "+2"], 'zticklabelrotation', 0 ) % ["-2", "Z[m]", "+2"] )
set( ani.hAxes{1},'LineWidth',3.5 ); set(ani.hAxes{1}, 'TickLength',[0.04 0.04]);

ani.run( 0.5, 3, true, ['output', num2str( idx ) ])

%% ==================================================================
%% (2-) [2021 DOSIM WORKSHOP] Video Geneartion
%% -- (2A) Reading the Datas

data = myTxtParse( './myData/for_explanation/data_log.txt' );

pSH_ZFT  = zeros( 3, length( data.currentTime ) ); 
pEL_ZFT  = reshape( pEL_func( 0.294, data.ZFT(1,:)', data.ZFT(2,:)'                                  ), [], 3 )';
pEE_ZFT  = reshape( pEE_func( 0.294, 0.291, data.ZFT(1,:)', data.ZFT(2,:)', data.ZFT(3,:)', data.ZFT(4,:)'), [], 3 )';

clear gObjs

stringList = [      "SH", "EL", "EE" ];                                


for i = 1 : length( stringList )
   gObjs( i  ) = myMarker( 'XData', data.geomXYZPositions( 3 * i - 2, :  ) , ... 
                           'YData', data.geomXYZPositions( 3 * i - 1, :  ) , ... 
                           'ZData', data.geomXYZPositions( 3 * i    , :  ) , ... 
                               'name', stringList( i ), ...
                           'SizeData',   700, ...
                          'LineWidth',   3            , ...
                    'MarkerEdgeColor',  c.black, 'MarkerFaceColor',  c.black ); 

end


gObjs( 4  ) = myMarker( 'XData', pEE_ZFT( 1, :  ) , ... 
                        'YData', pEE_ZFT( 2, :  ) , ... 
                        'ZData', pEE_ZFT( 3, :  ) , ... 
                               'name', "ZFT_EE", ...
                           'SizeData',   700, ...
                          'LineWidth',   3            , ...
                    'MarkerEdgeColor',  c.black, 'MarkerEdgeAlpha', 0.5, ...
                    'MarkerFaceColor',  c.black, 'MarkerFaceAlpha', 0.5); 
                
gObjs( 5  ) = myMarker( 'XData', pEL_ZFT( 1, :  ) , ... 
                        'YData', pEL_ZFT( 2, :  ) , ... 
                        'ZData', pEL_ZFT( 3, :  ) , ... 
                               'name', "ZFT_EL", ...
                           'SizeData',   700, ...
                          'LineWidth',   3            , ...
                    'MarkerEdgeColor',  c.black, 'MarkerEdgeAlpha', 0.5, ...
                    'MarkerFaceColor',  c.black, 'MarkerFaceAlpha', 0.5);                 
                
ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 


ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.black, 'LineStyle',  '-' );      
ani.connectMarkers( 1, [     "SH", "ZFT_EL", "ZFT_EE" ], 'Color', c.black, 'LineStyle',  '--' );      

% Add the 2nd figure plot
ani.adjustFigures( 2 );                     
ani.adjustFigures( 3 );                     

tmpC = [0.8500, 0.3250, 0.0980;
        0.9290, 0.6940, 0.1250;
        0.4940, 0.1840, 0.5560;
        0.4660, 0.6740, 0.1880];	

plot( ani.hAxes{ 2 }, data.currentTime, data.ZFT( 1, : ), 'color', tmpC( 1, : ) , 'linestyle', '--','linewidth', 5 );
tmp = myMarker( 'XData', data.currentTime , 'YData', data.qPos( 1, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  tmpC( 1, : )  ); 
ani.addTrackingPlots( 2, tmp );
    
plot( ani.hAxes{ 2 }, data.currentTime, data.ZFT( 4, : ), 'color', tmpC( 4, : ) , 'linestyle', '--','linewidth', 5 );
tmp = myMarker( 'XData', data.currentTime , 'YData', data.qPos( 4, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  tmpC( 4, : )  ); 
ani.addTrackingPlots( 2, tmp );
    
plot( ani.hAxes{ 2 }, data.currentTime, zeros( 1, length( data.currentTime) ), 'color', tmpC( 2, : ) , 'linestyle', '-','linewidth', 10 );
plot( ani.hAxes{ 2 }, data.currentTime, zeros( 1, length( data.currentTime) ), 'color', tmpC( 3, : ) , 'linestyle', '-','linewidth', 10 );

tmp = myMarker( 'XData', data.ZFT(1,:), 'YData', data.ZFT(4,:), 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  800, 'LineWidth', 6 , 'MarkerEdgeColor',  [0, 0.4470, 0.7410]  ); 
ani.addTrackingPlots( 3, tmp );             

tmpLim = 0.7;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0.0 0.0]     )           
%                      'view',   [41.8506   15.1025 ]     )      

set( ani.hAxes{ 2 }, 'XLim',   [ 0, 2.5] )

set( ani.hAxes{ 3 }, 'XLim',   [ -1.5 , 2.0] , ...                  
                     'YLim',   [ -1.5 , 2.0]   )           
%                      'view',   [41.8506   15.1025 ]     )      

ani.run( 0.5, 3, true, 'output' )

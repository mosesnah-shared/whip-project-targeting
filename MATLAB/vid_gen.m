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
myFigureConfig( 'fontsize', 30, ...
               'lineWidth', 10, ...
              'markerSize', 25    )  
             
global c                                                                   % Setting color structure 'c' as global variable
c  = myColor();            


%% ==================================================================
%% (1-) Video Geneartion
%% -- (1A) Calling the data for animation

idx  = 1;
% data = myTxtParse( ['./myData/simulation_log/data_log_T', num2str( idx ), '.txt' ] );
for i = 1 : 6
    data_raw{i} = myTxtParse( ['./myData/for_do_sim/data_log_T', num2str( i ), '.txt' ] );
end
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

idx = 6;
data = data_raw{ idx };

color_arr = [      0, 0.4470, 0.7410; ...
              0.8500, 0.3250, 0.0980; ...
              0.9290, 0.6940, 0.1250;...
              0.4940, 0.1840, 0.5560; ...
              0.4660, 0.6740, 0.1880; ...
              0.6350, 0.0780, 0.1840 ];


% For the target of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "target"  , ...
                     'SizeData',  500      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   color_arr( idx, : ) , ...
              'MarkerFaceColor',    color_arr( idx, : ), ...
              'MarkerFaceAlpha', 0.8       );                         % Defining the markers for the plot

          

stringList = [      "SH", "EL", "EE",     genNodes( N ) ];                                
sizeList   = [           500,      500,      500, 100 * ones( 1, N ) ];                            
colorList  = [ repmat( c.black, 3, 1 ); repmat( [0.75, 0.00, 0.75], N , 1 ) ];                            
           
% For the whole model
for i = 1 : length( stringList )
   gObjs( i + 1 ) = myMarker( 'XData', data.geomXYZPositions( 3 * i + 1, :  ) , ... 
                              'YData', data.geomXYZPositions( 3 * i + 2, :  ) , ... 
                              'ZData', data.geomXYZPositions( 3 * i + 3, :  ) , ... 
                               'name', stringList( i ), ...
                           'SizeData',   sizeList( i ), ...
                          'LineWidth',   4            , ...
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

%% ==================================================================
%% (3-) Force Vector vs. Power Plot
%% -- (3A) Reading the Datas


idx = 3;
data = myTxtParse( ['./myData/force_data/data_log_T', num2str( idx ), '_Force.txt' ] );

clear gObjs
% Marker in order, upper limb 3 joints( SH, EL, WR) 
genNodes = @(x) ( "node" + (1:x) );
N        = 25;


EEvel = data.geomXYZVelocities( 10:12, : );
fVec  = data.forceVec;
pIn = sum( fVec .* EEvel );

myC = [c.pink; c.blue; c.green ];          

% For the target of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "target"  , ...
                     'SizeData',  500      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   myC(idx, :) , ...
              'MarkerFaceColor',   myC(idx, :) , ...
              'MarkerFaceAlpha', 0.8       );                         % Defining the markers for the plot


stringList = [      "SH", "EL", "EE",     genNodes( N ) ];                                
sizeList   = [           700,      700,      700, 200 * ones( 1, N ) ];                            
colorList  = [ repmat( c.orange_milky, 3, 1 ); repmat( c.black, N , 1 ) ];                            
           
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

scale = 0.1;
gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData', data.geomXYZPositions( 12, : ), ...
                            'UData', scale * data.forceVec( 1, : ),  'VData', scale * data.forceVec( 2, : ), 'WData', scale * data.forceVec( 3, : ), 'LineWidth', 10, 'Color', c.black );              


ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array

ani.connectMarkers( 1, [     "SH",     "EL",     "EE" ], 'Color', c.grey, 'LineStyle',  '-' );      
% ani.connectMarkers( 1, [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ], 'Color', c.grey, 'LineStyle', '--' );      

tmpC = [ c.pink; c.green; c.blue; c.yellow ];

% Add the 2nd figure plot
ani.adjustFigures( 2 );                     
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



ani.addZoomWindow( 3 , "EE", 0.6 );   

set( ani.hAxes{ 1 }, 'LineWidth', 1.4 )
set( ani.hAxes{ 3 }, 'LineWidth', 1.4 )
set( ani.hAxes{ 3 }, 'Xticklabel',   [] , 'Yticklabel', [] , 'Zticklabel', [] ,'view',   [41.8506   15.1025 ]     )   

tmp1 = 40;

set( ani.hAxes{1}, 'xtick', [-2, 0, 2] ); set( ani.hAxes{1}, 'xticklabel', ["-2", "X (m)", "+2"], 'xticklabelrotation', 0 ) % ["-2", "X[m]", "+2"] )
set( ani.hAxes{1}, 'ytick', [-2, 0, 2] ); set( ani.hAxes{1}, 'yticklabel', ["-2", "Y (m)", "+2"], 'yticklabelrotation', 0 ) % ["-2", "Y[m]", "+2"] )
set( ani.hAxes{1}, 'ztick', [-2, 0, 2] ); set( ani.hAxes{1}, 'zticklabel', ["-2", "Z (m)", "+2"], 'zticklabelrotation', 0 ) % ["-2", "Z[m]", "+2"] )
set( ani.hAxes{1},'LineWidth',3.5 ); set(ani.hAxes{1}, 'TickLength',[0.04 0.04]);


tmp = myMarker( 'XData', data.currentTime , 'YData', pIn , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  [0, 0.4470, 0.7410]  ); 
ani.addTrackingPlots( 2, tmp );
    

tmp = myMarker( 'XData', data.currentTime , 'YData', data.minVal , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  400, 'LineWidth', 6 , 'MarkerEdgeColor',  [0.8500, 0.3250, 0.0980]  ); 
ani.addTrackingPlots( 2, tmp );


D = [0.95, 0.579, 0.95];
ttmp = find( data.minVal == min(data.minVal) );
set( ani.hAxes{2}, 'xtick', [0.1, 0.1+D( idx ), round( data.currentTime( ttmp  ), 2 ) ] )
set( ani.hAxes{2}, 'xlim', [0, max( data.currentTime ) ] )
legend( ani.hAxes{2}, 'Power', '', 'Dist.' )
ani.run( 0.5, 3, true, ['output', num2str( idx ) ])


%% ==================================================================
%% (4-) The cart and pole task
%% -- (4A) Reading the Datas

data = myTxtParse( './myData/cart_and_pole/data_log.txt' );

clear gObjs

% For the cart of the model
gObjs(  1 ) = myMarker( 'XData', data.geomXYZPositions( 1, :  ) , ... 
                        'YData', data.geomXYZPositions( 2, :  ) , ... 
                        'ZData', data.geomXYZPositions( 3, :  ) , ... 
                         'name', "cart"  , ...
                     'SizeData',  800      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   c.roseRed , ...
              'MarkerFaceColor',   c.roseRed );              
          
gObjs(  2 ) = myMarker( 'XData', data.geomXYZPositions( 4, :  ) , ... 
                        'YData', data.geomXYZPositions( 5, :  ) , ... 
                        'ZData', data.geomXYZPositions( 6, :  ) , ... 
                         'name', "blob"  , ...
                     'SizeData',  800      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   c.black , ...
              'MarkerFaceColor',   c.black );               
          
% Trim out the x0 and add it to gObjs.
data.x0 = data.x0_1( 3, :) .* data.alphas( 1, : ) + data.x0_2( 3, : ) .* data.alphas( 2, : );
          
gObjs(  3 ) = myMarker( 'XData', data.x0, ... 
                        'YData', zeros( 1, length( data.currentTime ) )  , ... 
                        'ZData', zeros( 1, length( data.currentTime ) ) , ... 
                         'name', "x0"  , ...
                     'SizeData',  400      , ...
                    'LineWidth',   1       , ...
              'MarkerEdgeColor',   c.grey , ...
              'MarkerFaceColor',   c.grey , ...
              'MarkerFaceAlpha',  0.5 );               

ani = myAnimation( data.currentTime( 2 ), gObjs );                         % Input (1) Time step of sim. 
                                                                           %       (2) Graphic Objects (Heterogeneouus) Array
ani.connectMarkers( 1, [ "cart", "blob" ], 'Color', c.grey, 'LineStyle',  '-' );   

tmpLim = 2.3;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0, 0 ]     )    
set( ani.hAxes{1}, 'xtick', [-2, 0, 2] );
set( ani.hAxes{1}, 'ytick', [-2, 0, 2] );
set( ani.hAxes{1}, 'ztick', [-2, 0, 2] );
ani.adjustFigures( 2 );                     

% plot( ani.hAxes{ 2 }, data.qPos( 2, : ), data.qVel( 2, : ), 'linewidth', 3, 'linestyle', '--' , 'color', c.pink ) 
tmp = myMarker( 'XData', data.qPos( 2, : ) , 'YData', data.qVel( 2, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  500, 'LineWidth', 4 , 'MarkerEdgeColor',  c.purple  ); 
ani.addTrackingPlots( 2, tmp );
% xlabel( ani.hAxes{ 2 }, "Pole Ang. Pos." )
% ylabel( ani.hAxes{ 2 }, "Pole Ang. Vel." )

ani.adjustFigures( 3 );        

tmp1 = myMarker( 'XData', data.currentTime , 'YData', data.alphas( 1, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  500, 'LineWidth', 4 , 'MarkerEdgeColor',  c.yellow  ); 
tmp2 = myMarker( 'XData', data.currentTime , 'YData', data.alphas( 2, : ) , 'ZData', zeros( 1, length( data.currentTime ) ), ...
                 'SizeData',  500, 'LineWidth', 4 , 'MarkerEdgeColor',  c.blue  ); 
D = 2.72;
patch([D 4.5 4.5 D],[0 0 2 2], c.green, 'facealpha', 0.4 )

set( ani.hAxes{ 3 }, 'XLim',   [ 0 , 4.5 ] , ...                  
                     'YLim',   [ 0 , 1.5 ] )    
ani.addTrackingPlots( 3, tmp1 );
ani.addTrackingPlots( 3, tmp2 );

ani.run( 1, 4.5, true, 'cart_and_pole')

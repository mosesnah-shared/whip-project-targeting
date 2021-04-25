% [Project]        [M3X] Whip Project
% [Title]          Sample script for animations
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

% [Moses Nah] [Backup] To calculate the time differentiation of the jacobian matrix
dJ = diff( J, sym( 't' ) );
dJ = subs( dJ, diff( robot.q, robot.t ), robot.dq );             % Simple substitution

old = [ "q1(t)", "q2(t)", "dq1(t)", "dq2(t)" ]; new = [ "q1", "q2", "dq1", "dq2" ];

 J_func = myFunctionize(  J, old, new );
dJ_func = myFunctionize( dJ, old, new );

%% -- (1B) PYTHON Conversion process
%       Skip this section if you are not interested in PYTHON implementation
%       In case you are using python, we need to copy-and-paste the Y matrix..
%       This section does the job for you.

if     robot.nDOF == 4
    oldS = [ string( [ robot.q, robot.dq, robot.L ] ), "sin", "cos", "^2"  ];                            % oldS: MATLAB Style print,   
    newS = [ "q[0]", "q[1]", "q[2]", "q[3]", "dq[0]","dq[1]","dq[2]","dq[3]", "L1", "L2", "np.sin", "np.cos", "**2" ];   % newS: python Style print                                     
     
elseif robot.nDOF == 2
    oldS = [ string( [ robot.q, robot.dq, robot.L ] ), "sin", "cos", "^2"  ];                            % oldS: MATLAB Style print,   
    newS = [ "q[0]", "q[1]", "dq[0]","dq[1]", "L1", "L2", "np.sin", "np.cos", "**2" ];   % newS: python Style print                                         
    
end
    
myPythonConversion( C, "C", oldS, newS )     


%% ==================================================================
%% (2-) Running the simulation in MATLAB
%          [WARNING] If you run for 4DOF robot case, it is extremely slow!
%% -- (2A) Calculate the M, C matrix and G vector, as a function of q

%                                                   Straighten the I matrix to a row vector
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

pEL_func = myFunctionize( pEL_sym, old, new );
pEE_func = myFunctionize( pEE_sym, old, new );



%% -- (2B) Calling the data for animation 

idx  = 3;
data = myTxtParse( ['./myData/data_log_T', num2str( idx ), '.txt' ] );

% data = myTxtParse( "./myData/data_log_T3_half.txt" );

%% -- (2C) (For Joint Impedance Controller) Run the animation
% Since ode45 varies the step size, changing it to equivalent step size
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

set( ani.hAxes{ 1 }, 'LineWidth', 1.4 )
set( ani.hAxes{ 3 }, 'LineWidth', 1.4 )


tmp1 = 40;

xlabel( ani.hAxes{ 1 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 1 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 1 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 3 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 3 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 3 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 2 }, 'Time [sec]', 'Fontsize', 30 ); ylabel( ani.hAxes{ 2 }, '$q, q_0$ [rad]', 'Fontsize', tmp1);
set( ani.hAxes{ 2 }, 'LineWidth', 1.4, 'XLim', [0, 3] )

ani.run( 0.33, 2.3, true, ['output', num2str( idx ) ])


%% -- (2D) (For Cartesian Impedance Controller) Run the animation

idx = 1;
data = myTxtParse( './myData/data_log.txt');

clear gObjs

% Marker in order, upper limb 3 joints( SH, EL, WR) 
genNodes = @(x) ( "node" + (1:x) );
N        = 10;

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

% stringList = [      "SH",   "EL",  "EE" ];                                
% sizeList   = [       400,   400,   400  ];                            
% colorList  = [ repmat( c.green, 3, 1 )  ];                            


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

% For the ZFT end-effector position
gObjs( end + 1 ) = myMarker( 'XData', data.x0( 2, :  ) , ... 
                             'YData', data.x0( 1, :  ) , ... 
                             'ZData', data.x0( 3, :  ) , ... 
                              'name', stringList( i ), ...
                          'SizeData',   sizeList( i ), ...
                         'LineWidth',   7            , ...
                   'MarkerEdgeColor',  c.peach ); 

delX  = - data.geomXYZPositions( 10:12, : )  + data.x0( [2,1,3], : );
deldX = - data.geomXYZVelocities( 10:12, : ) + data.dx0( [2,1,3], : );



Kx = 5 * eye(3);  Bx = Kx; 

Fk = Kx * delX;   Fb = Bx * deldX;

F = Fk + Fb;

gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData', data.geomXYZPositions( 12, : ), ...
                            'UData', data.forceVec( 1, : ),          'VData', data.forceVec( 2, : )                 , 'WData', data.forceVec( 3, : ), 'LineWidth', 10, 'Color', c.blue );              
                        
gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData', data.geomXYZPositions( 12, : ), ...
                            'UData', Fk( 1, : ),          'VData', Fk( 2, : )                 , 'WData', Fk( 3, : ), 'LineWidth', 3, 'Color', c.green );                                      
                        
gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData', data.geomXYZPositions( 12, : ), ...
                            'UData', Fb( 1, : ),          'VData', Fb( 2, : )                 , 'WData', Fb( 3, : ), 'LineWidth', 3, 'Color', c.green );                                                              
                        
gObjs( end + 1 ) = myArrow( 'XData', data.geomXYZPositions( 10, : ), 'YData', data.geomXYZPositions( 11, : ), 'ZData', data.geomXYZPositions( 12, : ), ...
                            'UData', F( 1, : ),          'VData', F( 2, : )                 , 'WData', F( 3, : ), 'LineWidth', 10, 'Color', c.orange );                                                                                      

% For the ZFT Postures
% Forward Kinematics, the ZFT Positions

% pSH_ZFT  = zeros( 3, length( data.currentTime ) ); 
% pEL_ZFT  = reshape( pEL_func( data.pZFT(1,:)', data.pZFT(2,:)'                                  ), [], 3 )';
% pEE_ZFT  = reshape( pEE_func( data.pZFT(1,:)', data.pZFT(2,:)', data.pZFT(3,:)', data.pZFT(4,:)'), [], 3 )';
% 
% pos = { pSH_ZFT, pEL_ZFT, pEE_ZFT };
%  
% 
% stringList = [ "SH_ZFT", "EL_ZFT", "EE_ZFT" ];                                
% sizeList   = [      400,      400,      400 ];                            
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

tmpLim = 0.8;
set( ani.hAxes{ 1 }, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0   0]     )           
%                      'view',   [41.8506   15.1025 ]     )                


ani.addZoomWindow( 3 , "EE", 0.6 );   
set( ani.hAxes{ 3 },  'view',   [41.8506   15.1025 ]     ) 

set( ani.hAxes{ 1 }, 'LineWidth', 1.4 )
set( ani.hAxes{ 3 }, 'LineWidth', 1.4 )


tmp1 = 40;

xlabel( ani.hAxes{ 1 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 1 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 1 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 3 },      'X [m]', 'Fontsize', tmp1 ); ylabel( ani.hAxes{ 3 }, 'Y [m]', 'Fontsize', tmp1); zlabel( ani.hAxes{ 3 }, 'Z [m]', 'Fontsize', tmp1);
xlabel( ani.hAxes{ 2 }, 'Time [sec]', 'Fontsize', 30 ); ylabel( ani.hAxes{ 2 }, '$q, q_0$ [rad]', 'Fontsize', tmp1);
set( ani.hAxes{ 2 }, 'LineWidth', 1.4, 'XLim', [0, 3] )



ani.run( 0.33, 8, false, ['output', num2str( idx ) ])

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
myFigureConfig( 'fontsize', 40, ...
               'lineWidth', 10, ...
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

%% ==================================================================
%% (3-) The modal analysis of the whip
%% -- (3A) Calculation of the eigenvalues/eigenvectors, Pole-Zero Plot

% Defining the whip parameters of the whip model.
tmp = num2cell( [25, 0.012, 0.072, 0.242, 0.092] );                        % The whip parameters
[N, m, l, k, b] = deal( tmp{ : } );

% N, m, l, k, b = [25, 0.012, 0.072, 0.242, 0.000];
% N, m, l, k, b = [25, 0.012, 0.072, 0.242, 0.092];

g = 9.81;

% Capital letters denote the matrix of the parameters 

if N == 1   % If it is a single pendulum, 
    % Then the equation of motion is as follows:
    % ml^2 q'' + bq' + (k + mgl)q = tau
    K = k / l;
    M = m * l;
    G = m * g;
    B = b / l;
else
    tmp = toeplitz([2 -1 zeros(1 , N - 2)]);
    TK = tmp;
    tmp(end, end) = 1;
    TK( 1, 1 ) = 1;
    M = m * l * inv( TK );
    K = k / l * tmp;
    B = b / l * tmp;
    G = m * g * diag( fliplr( 1 : N ) ) ;
end

% Original eigenvalue/eigenvector
[eigV, eigD]  = eig( [ zeros( N ), eye( N );
               -inv( M ) * ( K + G ), -inv( M ) * B     ]);                % Calculating the eigenvalues of the statespace matrix

% Canonical form of the (complex) eigenvalue/eigenvector problem
[eigV_c, eigD_c] = cdf2rdf( eigV, eigD );
% The diagonal part is the "dissipative" elements, and the off-diagonal part is the "oscillatory" elements.

tmpEig = diag( eigD ); 

% Extract-out eigenvalues with imaginary parts 
idx = find( imag( tmpEig ) ~= 0 );      % Getting the idxs of the complex eigenvalues
cpxEig = tmpEig( idx' );                % Extracting out the complex eigenvalues
N = length( cpxEig ) / 2;               % The number of distinct complex eigenvalues


colorArr = flipud([c.blue; c.green; c.pink]);
% colorArr = flipud( colormap( copper( 2 * N ) ) );

tmp1 = [12.31, 6.856, 2.823];

f = figure( );
a = axes( 'parent', f );
hold on
tmpS = [ "*", "d", "o"];


th = linspace( pi/2, 3 * pi/2, 100 );

for i = 1 :  N
    cpx = cpxEig( 2 * i - 1 );
    scatter3( real( cpx ),  imag( cpx ), 0, 400, tmpS{ i },  ...
                    'MarkerFaceColor', colorArr( i, :), ...
                         'MarkerEdgeColor', colorArr( i, :), ...
                         'LineWidth',4.5, 'parent', a )
    scatter3( real( cpx ), -imag( cpx ), 0,400, tmpS{ i },  ...
                     'MarkerFaceColor', colorArr( i, :), ...
                         'MarkerEdgeColor', colorArr( i, :), ...
                          'LineWidth',4.5, 'parent', a )    
                                   

    R( i ) = sqrt( real( cpx ).^2 + imag( cpx ).^2 );  %or whatever radius you want
    x = R(i) * cos(th);
    y = R(i) * sin(th);
    plot(x,y, 'linewidth', 1.3, 'linestyle', '--', 'color', colorArr( i, : ) );                       
                    
    text( -R( i )+0.3, -0.8, num2str( -R( i ), '%.2f' ), 'fontsize', 23, 'color', colorArr( i, : ) ) 
    scatter3( -R( i ),     0, 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', colorArr( i, :), 'linewidth', 2 )
    if i == 3
        continue
    end    
        scatter3(       0, -R(i), 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', colorArr( i, :), 'linewidth', 2 )
        scatter3(       0,  R(i), 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', colorArr( i, :), 'linewidth', 2 )
    
end

tmpL = 14;
mxAxis = mArrow3( [ -tmpL, 0, -1], [ 2, 0   , -1 ], 'color', 0.6 * ones( 1,3 ), 'stemWidth', 0.05, 'tipWidth', 0.3 );
myAxis = mArrow3( [ 0, -tmpL-1, -1], [ 0, tmpL+1, -1 ], 'color', 0.6 * ones( 1,3 ), 'stemWidth', 0.05, 'tipWidth', 0.3 );

text( 1, -1.5,   texlabel('Real'), 'fontsize', 30 )
text( 0.3, tmpL, texlabel('Imag'), 'fontsize', 30 )

text( -24, 12, '$-6.39 \pm 11.26j$', 'fontsize', 30, 'interpreter',  'latex','color', colorArr( 1, : ))
text( -24, 10, '$-0.89 \pm  6.910j$', 'fontsize', 30, 'interpreter', 'latex','color', colorArr( 2, : ))
text( -24, 8,  '$-0.03 \pm  2.830j$', 'fontsize', 30, 'interpreter', 'latex','color', colorArr( 3, : ))

scatter3( -25, 12, 0.1, 400, '*', 'markerfacecolor', colorArr( 1, : ), 'markeredgecolor', colorArr( 1, :), 'linewidth', 4.5 )
scatter3( -25, 10, 0.1, 400, 'd', 'markerfacecolor', colorArr( 2, : ), 'markeredgecolor', colorArr( 2, :), 'linewidth', 4.5 )
scatter3( -25,  8, 0.1, 400, 'o', 'markerfacecolor', colorArr( 3, : ), 'markeredgecolor', colorArr( 3, :), 'linewidth', 4.5 )

tmpL = 17;
set( a, 'ylim', [-tmpL, tmpL] );
set( a, 'xlim', [-tmpL, tmpL] );
axis equal off
grid on

% mySaveFig( gcf, 'S1_PZPlot' )
exportgraphics( f,'S1_PZPlot.pdf','ContentType','vector')

%% -- (3B) EigenMode Plot

% Defining the whip parameters of the whip model.
tmp = num2cell( [25, 0.012, 0.072, 0.242, 0.0] );                        % The whip parameters
[N, m, l, k, b] = deal( tmp{ : } );

if N == 1   % If it is a single pendulum, 
    % Then the equation of motion is as follows:
    % ml^2 q'' + bq' + (k + mgl)q = tau
    K = k / l;
    M = m * l;
    G = m * g;
    B = b / l;
else
    tmp = toeplitz([2 -1 zeros(1 , N - 2)]);
    TK = tmp;
    tmp(end, end) = 1;
    TK( 1, 1 ) = 1;
    M = m * l * inv( TK );
    K = k / l * tmp;
    B = b / l * tmp;
    G = m * g * diag( fliplr( 1 : N ) ) ;
end

% Original eigenvalue/eigenvector
[eigV, eigD]  = eig( [ zeros( N ), eye( N );
               -inv( M ) * ( K + G ), -inv( M ) * B     ]);                % Calculating the eigenvalues of the statespace matrix

% Canonical form of the (complex) eigenvalue/eigenvector problem
[eigV_c, eigD_c] = cdf2rdf( eigV, eigD );
% The diagonal part is the "dissipative" elements, and the off-diagonal part is the "oscillatory" elements.

tmpEig = diag( eigD ); 

% For damping b = 0, The last 6 vectors are the eigenvectors.
if b == 0
    eigvec = eigV( :, 45 : end );
    eig_mode = eigvec(1:N, : );
end

% Amplifying Factor
alpha = [8, 20, 30];
% alpha = 18;   % 5, 12

f = figure( );
a = axes( 'parent', f );
hold on


eigmode3 = alpha( 3 ) * imag( eig_mode( :, 1 ) );
eigmode2 = alpha( 2 ) * imag( eig_mode( :, 3 ) );
eigmode1 = alpha( 1 ) * imag( eig_mode( :, 5 ) );

nPoints = 100;

eigmode3_span = eigmode3 - 2 * eigmode3 * log( linspace( exp(0), exp(1), nPoints) );
eigmode2_span = eigmode2 - 2 * eigmode2 * log( linspace( exp(0), exp(1), nPoints) );
eigmode1_span = eigmode1 - 2 * eigmode1 * log( linspace( exp(0), exp(1), nPoints) );

eigmode3_span = [eigmode3_span, fliplr( eigmode3_span ) ];
eigmode2_span = [eigmode2_span, fliplr( eigmode2_span ) ];
eigmode1_span = [eigmode1_span, fliplr( eigmode1_span ) ];


xPos_mode1 =  l * cumsum( sin( eigmode1_span ) ) - 2; 
zPos_mode1 = -l * cumsum( cos( eigmode1_span ) );

xPos_mode2 =  l * cumsum( sin( eigmode2_span ) );
zPos_mode2 = -l * cumsum( cos( eigmode2_span ) );

xPos_mode3 =  l * cumsum( sin( eigmode3_span ) ) + 2;
zPos_mode3 = -l * cumsum( cos( eigmode3_span ) );


p1  = plot( xPos_mode1(:,1), zPos_mode1(:,1), '-', 'color', c.blue  , 'linewidth', 8 );
p2  = plot( xPos_mode2(:,1), zPos_mode2(:,1), '-', 'color', c.green , 'linewidth', 8 );
p3  = plot( xPos_mode3(:,1), zPos_mode3(:,1), '-', 'color', c.pink  , 'linewidth', 8 );

sp1 = scatter( xPos_mode1(:,1), zPos_mode1(:,1), 350, 'o', 'markeredgecolor', c.blue , 'markerfacecolor', c.white, 'linewidth', 3 );
sp2 = scatter( xPos_mode2(:,1), zPos_mode2(:,1), 350, 'o', 'markeredgecolor', c.green, 'markerfacecolor', c.white, 'linewidth', 3 );
sp3 = scatter( xPos_mode3(:,1), zPos_mode3(:,1), 350, 'o', 'markeredgecolor', c.pink , 'markerfacecolor', c.white, 'linewidth', 3 );

set( gca, 'ylim', [-2,0], 'xlim', [-3.5, 3.5] )
set( gca, 'xticklabel', [], 'yticklabel', [] )
set( gca,'LineWidth',3) 


    % axis box

exportgraphics( f,'S2_ModeShape.pdf','ContentType','vector')


%% ==================================================================
%% (4-) Plot for iteration vs. Optimization
%% -- (4A) Calling the data + Plot

data1 = myTxtParse( './myData/optimization_process/optimization_log_T1.txt');
data2 = myTxtParse( './myData/optimization_process/optimization_log_T2.txt');
data3 = myTxtParse( './myData/optimization_process/optimization_log_T3.txt');

f = figure(  );
a = axes( 'parent', f );

hold on 

plot( data1.Iter, data1.output, 'linewidth', 5, 'color', c.pink  );
plot( data2.Iter, data2.output, 'linewidth', 5, 'color', c.blue  );
plot( data3.Iter, data3.output, 'linewidth', 5, 'color', c.green );

set( gca,'LineWidth',3) 
xlabel( 'Iteration [-]' ), ylabel( '$L^*$ [m]' );
legend( 'Target 1', 'Target 2', 'Target 3', 'fontsize', 40 )

exportgraphics( f,'F2_iter_and_optimization.pdf','ContentType','vector')


%% ==================================================================
%% (5-)
%% -- (5A) Calling the data + Plot

for i = 1 : 3
    
   rawData{ i } = myTxtParse( ['myData/simulation_log/data_log_T', num2str( i ), '.txt']  );
%     rawData{ i } = myTxtParse( ['data_log_dense_T', num2str( i ), '.txt']  );
   
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );
   
end

%% -- (5B) Plot of Movement Snapshots

idx = 3;        % Choose Target Type

idxS = find( rawData{ idx }.outputVal == min( rawData{ idx }.outputVal )  );

tIdx = [10, 68, idxS;
        10, 37, idxS; 
        10, 61, idxS];

% viewArr = [ 49.9456, 4.7355;
%             68.8342, 6.0279;
%             44.3530, 7.4481];    

viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355 ];

alpha = [0.2, 0.5, 1.0];                                              % The alpha values of each screen shot   
f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;

switch idx 
   
    case 1
        cTarget = c.pink;

    case 2
        cTarget = c.blue;
    case 3
        cTarget = c.green;
end

mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 400, ...        % Setting the handle of the ZFT Plot, 
                   'parent', a,   'LineWidth', 5,               ...       % For the main plot (s1)
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', cTarget, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );

      
for i = 1 : length( tIdx )
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ c.orange_milky, alpha( i ) ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), 800, ... 
                   'parent', a,   'LineWidth', 4, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.orange_milky, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );

               
%     p3 = plot3(  rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), ...
%                  'parent', a, ...
%                 'linewidth', 8, 'color', [ c.purple_plum, alpha( i ) ] );
            
    p4 = scatter3( rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), 100, ... 
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.purple_plum, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );
               
               
end    


tmpLim = 2.4;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : ) )  %  [BACKUP] [Target #1] 16.3213    6.0865
      

set(a,'LineWidth',3.5 )%,'TickLength',[0.025 0.025]);
set( a, 'xtick', [-2, 0, 2] ); set( a, 'xticklabel', ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-2, 0, 2] ); set( a, 'yticklabel', ["-2", "Y[m]", "+2"] )
set( a, 'ztick', [-2, 0, 2] ); set( a, 'zticklabel', ["-2", "Z[m]", "+2"] )


exportgraphics( f,['F3_',num2str(idx),'_timelapse.pdf'],'ContentType','vector')


% mySaveFig( f, ['output', num2str( idx )] );

%% -- (5C) The end-effector and the elbow's trajectory 

% Plotting the ``trace'' or ``path'' of the upper-limb movement.
idx = 3;

switch idx 
   
    case 1
        tStart = 0.1; D = 0.950; % tStart = 0.3 if not Dense!
    case 2
        tStart = 0.1; D = 0.579;
    case 3
        tStart = 0.1; D = 0.950;
end

idxS = find( rawData{ idx }.currentTime >= tStart & rawData{ idx }.currentTime <= tStart + D );	
% idxS = idxS( 1 : 3 : end);
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;

scatter3( rawData{ idx }.geomXPositions( 2, idxStart ), ...
          rawData{ idx }.geomYPositions( 2, idxStart ), ...
          rawData{ idx }.geomZPositions( 2, idxStart ), 2000, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.orange_milky, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );


plot3( rawData{ idx }.geomXPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 3, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 4, 'color', [c.blue, 0.8] )

plot3( rawData{ idx }.geomXPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 4, idxStart : idxEnd ), ...
      'parent', a,   'LineWidth', 4, 'color', [c.green, 0.8] )  

alpha = linspace( 0.1, 1, length( idxS ) );
lwArr = linspace( 1, 8, length( idxS ) );
mStart = [ 1,  2,   1 ];
mStep  = [ 11, 8,  11 ];
mEnd   = [ 57, 35, 57 ];
for i = mStart( idx ) : mStep( idx ) : mEnd( idx )
    
    plot3( rawData{ idx }.geomXPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomYPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomZPositions( 2:4, idxS( i ) ), ...
                          'parent', a,   'LineWidth', lwArr( i ), 'color', [ c.orange_milky, alpha( i ) ]  )    
    
    scatter3( rawData{ idx }.geomXPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 3, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 4, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.orange_milky , ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i) );           
        
           
    scatter3( rawData{ idx }.geomXPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 4, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 4, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.orange_milky,  ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i) );           

end


xEL = rawData{ idx }.geomXPositions( 3, idxS )';
yEL = rawData{ idx }.geomYPositions( 3, idxS )';
zEL = rawData{ idx }.geomZPositions( 3, idxS )';

xEE = rawData{ idx }.geomXPositions( 4, idxS )';
yEE = rawData{ idx }.geomYPositions( 4, idxS )';
zEE = rawData{ idx }.geomZPositions( 4, idxS )';

[ kEL, volEL ] = convhull( xEL, yEL, zEL, 'Simplify',true );
[ kEE, volEE ] = convhull( xEE, yEE, zEE, 'Simplify',true );

% xlabel( 'X [m]', 'fontsize', 50 ); 
% ylabel( 'Y [m]', 'fontsize', 50 );
% zlabel( 'Z [m]', 'fontsize', 50 );

% viewArr = [ 49.9456, 4.7355;
%             68.8342, 6.0279;
%             44.3530, 7.4481];    

viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355 ];

tmpLim = 0.6;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : )   )  
      
set(a,'LineWidth',3.5,'TickLength',[0.025 0.025]);
      
set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["-0.5", "X[m]", "+0.5"] )
set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["-0.5", "Y[m]", "+0.5"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["-0.5", "Z[m]", "+0.5"] )

               
% trisurf( kEL, xEL, yEL, zEL, 'Facecolor',  c.blue, 'FaceAlpha', 0.1, 'EdgeColor', 'none' );
% trisurf( kEE, xEE, yEE, zEE, 'Facecolor', c.green, 'FaceAlpha', 0.1, 'EdgeColor', 'none' );

% xEE = d3.geomXYZPositions( 10, : );
% yEE = d3.geomXYZPositions( 11, : );
% zEE = d3.geomXYZPositions( 12, : );
% 
% [xx, yy] = meshgrid( xEE, yEE );
% C = planefit( xEE, yEE, zEE );
% zzft = C( 1 ) * xx + C( 2 ) * yy + C( 3 );
% surf( xx, yy, zzft, 'edgecolor', 'none', 'facecolor', c.green, 'facealpha', 0.3 )
% hold on
% plot3( xEE, yEE, zEE, 'o' );
% axis equal
% p3 = plot3(  rawData{ idx }.geomXPositions( 5:end, tIdx( i ) ), ...
%              rawData{ idx }.geomYPositions( 5:end, tIdx( i ) ), ...
%              rawData{ idx }.geomZPositions( 5:end, tIdx( i ) ), ...
%              'parent', a, ..
%             'linewidth', 6, 'color', [ c.purple_plum, alpha( i ) ] );

% mySaveFig( f, ['output', num2str( idx )] );
exportgraphics( f,['F4_',num2str(idx),'_timelapse_EL_EE.pdf'],'ContentType','vector')

%% -- (5D) Best-fit-Plane Identification/Calculation


% Plotting the ``trace'' or ``path'' of the upper-limb movement.
idx  = 3;
idx2 = 2;       % 1: EL, 2: EE 

switch idx 
   
    case 1
        tStart = 0.3; D = 0.950; % tStart = 0.3 if not Dense!
        color = c.pink;
    case 2
        tStart = 0.3; D = 0.579;
        color = c.blue;
    case 3
        tStart = 0.3; D = 0.950;
        color = c.green;
end




% switch idx2 
%         
%     case 1  %% EL
%           color = c.blue;
%     case 2  %% EE
%           color = c.green;
% end

idxS = find( rawData{ idx }.currentTime >= tStart & rawData{ idx }.currentTime <= tStart + D );	
idxStart = min( idxS ); idxEnd = max( idxS );



f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;

x = rawData{ idx }.geomXPositions( idx2 + 2, idxS )';
y = rawData{ idx }.geomYPositions( idx2 + 2, idxS )';
z = rawData{ idx }.geomZPositions( idx2 + 2, idxS )';

p  = [ x, y, z ];
pC = mean( p );

pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig(pn' * pn);

[ eigvecs2, ~, eigvals2 ] = pca( p );                                        % Running the PCA of the data

% The last vector of eigvecs correspond to the normal vector of the plane, which is the smallest pca value of the data matrix
w = null( eigvecs( : , 1)' );                                              % Find two orthonormal vectors which are orthogonal to v


scatter3( rawData{ idx }.geomXPositions( 2, idxStart ), ...
          rawData{ idx }.geomYPositions( 2, idxStart ), ...
          rawData{ idx }.geomZPositions( 2, idxStart ), 300, 's', ... 
          'parent', a,   'LineWidth', 4, ...
          'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.orange_milky, ...
          'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

plot3( rawData{ idx }.geomXPositions( idx2 + 2, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( idx2 + 2, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( idx2 + 2, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 4, 'color', [color, 0.3] )

scatter3(  rawData{ idx }.geomXPositions( idx2 + 2, idxS ), ...
           rawData{ idx }.geomYPositions( idx2 + 2, idxS ), ...
           rawData{ idx }.geomZPositions( idx2 + 2, idxS ), 200, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           
       

tmpLim = 0.45;      

tmp = 0;
for i = 1 : length( idxS )  % Brute force calculation of the distance.
    ttmp = abs( eigvecs(1,1) * ( x(i) - pC(1) ) + eigvecs(2,1) * ( y(i) - pC(2) ) + eigvecs(3,1) * ( z(i) - pC(3) )  ) ;
    tmp = tmp + ttmp * ttmp;
end

sqrt( tmp/length( idxS ) )

[P,Q] = meshgrid( -tmpLim: 0.1 : tmpLim );                              % Provide a gridwork (you choose the size)

XX = pC( 1 ) + w(1,1) * P + w(1,2) * Q;                                    % Compute the corresponding cartesian coordinates
YY = pC( 2 ) + w(2,1) * P + w(2,2) * Q;                                    %   using the two vectors in w
ZZ = pC( 3 ) + w(3,1) * P + w(3,2) * Q;
       
scatter3(  pC( 1 ) , pC(2), pC(3), 400, 'd',... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           

if idx == 1       
    mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', color, 'tipWidth', 0.01, 'stemWidth', 0.004 )
else    
    mArrow3( pC, pC + 0.3 * eigvecs( : , 1 )', 'color', color, 'tipWidth', 0.01, 'stemWidth', 0.004 )
end
% mArrow3( pC, pC + 0.3 * eigvecs( : , 2 )', 'colorcl', c.green, 'tipWidth', 0.01, 'stemWidth', 0.004 )
% mArrow3( pC, pC + 0.3 * eigvecs( : , 3 )', 'color', c.green, 'tipWidth', 0.01, 'stemWidth', 0.004 )

surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', color, 'facealpha', 0.3 )
tmpLim2 = 0.7;

viewArr = [99.3451, 5.0653;
          142.4901, 3.2252;
          133.9720    3.2060];
%           126.8750, 3.20293];

set( a,   'XLim',   [ - tmpLim2, tmpLim2 ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim2, tmpLim2 ] , ...    
          'ZLim',   [ - tmpLim2, tmpLim2 ] , ...
          'view',   viewArr( idx, : ) )  

if idx == 1 
    set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["", "X[m]", ""] )
else
    set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["-0.5", "X[m]", "+0.5"] )
end
    
set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["-0.5", "Y[m]", "+0.5"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["-0.5", "Z[m]", "+0.5"] )      

xtickangle( 0 ); ytickangle( 0 )

exportgraphics( f,['F4_',num2str(idx),'_best_fit_plane.pdf'],'ContentType','vector')

%% -- (5E) Contribution of each Movements


K = [ 17.4,  4.7, -1.90, 8.40; ...
      9.00, 33.0,  4.40, 0.00; ...
     -13.6,  3.0, 27.70, 0.00; ...
      8.40,  0.0,  0.00, 23.2];
  
Ksym = ( K + K' )/2;      % Extracting out the symmetric part

[V, D] = eig( Ksym );

v1 = V( :,1 );
v2 = V( :,2 );
v3 = V( :,3 );
v4 = V( :,4 );      % Ordered in ascending order of the size of eigenvalues. 

idx = 3;

switch idx 
   
    case 1
        tStart = 0.3; D = 0.950; % tStart = 0.3 if not Dense!
    case 2
        tStart = 0.3; D = 0.579;
    case 3
        tStart = 0.3; D = 0.950;
end

idxS = find( rawData{ idx }.currentTime >= tStart & rawData{ idx }.currentTime <= tStart + D );	
idxStart = min( idxS ); idxEnd = max( idxS );

clear c1 c2 c3 c4

    
dp = rawData{ idx }.jointAngleActual( 1:4, : ) - rawData{ idx }.pZFT;
dv =   rawData{ idx }.jointVelActual( 1:4, : ) - rawData{ idx }.vZFT;

c1_K = dp' * v1;
c2_K = dp' * v2;
c3_K = dp' * v3;
c4_K = dp' * v4;

c1_B = dv' * v1;
c2_B = dv' * v2;
c3_B = dv' * v3;
c4_B = dv' * v4;


% norm( c1_K( idxS ), 2 )
% norm( c2_K( idxS ), 2 )
% norm( c3_K( idxS ), 2 )
% norm( c4_K( idxS ), 2 )


f = figure( ); a = axes( 'parent', f );hold on;

plot( rawData{idx}.currentTime( idxS ) - tStart, c1_K( idxS ), 'linewidth', 15, 'linestyle', '-' )
plot( rawData{idx}.currentTime( idxS ) - tStart, c2_K( idxS ), 'linewidth', 15, 'linestyle', '--' )
plot( rawData{idx}.currentTime( idxS ) - tStart, c3_K( idxS ), 'linewidth', 15, 'linestyle', ':' )
plot( rawData{idx}.currentTime( idxS ) - tStart, c4_K( idxS ), 'linewidth', 15, 'linestyle', '-.' )
% plot( rawData{idx}.currentTime, [c1_K, c2_K, c3_K, c4_K]' )
% plot( rawData{idx}.currentTime, [c1_B, c2_B, c3_B, c4_B]' )
legend( "$c_1$","$c_2$","$c_3$","$c_4$", 'fontsize', 50, 'location', 'northwest' );

set( a,   'XLim',   [ 0, rawData{idx}.currentTime( idxEnd ) - tStart ], 'fontsize', 40 )
set( a,   'YLim',   [ -1, 1]                                          , 'fontsize', 40 )
set( a, 'ytick', [-1, -0.5, 0, 0.5, 1], 'yticklabel', ["-1", "", "0", "", "1"] )
xlabel( 'Time [sec]'      , 'fontsize', 50 ); 
ylabel( 'Contribution [-]', 'fontsize', 50 );


exportgraphics( f,['F5_',num2str(idx),'_contribution.pdf'],'ContentType','vector')

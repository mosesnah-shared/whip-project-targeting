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
val_array = { 1.595, 0.869 };  % Actual Length of robot
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

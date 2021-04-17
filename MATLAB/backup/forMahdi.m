% [Project]        [M3X] Whip Project
% [Title]          Script for Mahdi
% [Author]         Moses C. Nah
% [Creation Date]  Monday, April 15th, 2021
%
% [Emails]         Moses C. Nah   : mosesnah@mit.edu

%% (--) INITIALIZATION

clear all; close all; clc; workspace;

cd( fileparts( matlab.desktop.editor.getActiveFilename ) );                % Setting the current directory (cd) as the current location of this folder. 

%% (1-) Call Data

load forMahdi

% 150 iterations were conducted for the optimization. 
% Hence, for i of wholeData( i ), it contains the i-th trial of the simulation.
% [FOCUS] The data was sorted so that as the index goes higher, the better the performance. 
% i.e., the whip tip got closer as the index of wholeData increases.
% i.e., performance of wholeData(1) was worse, and wholeData(150) was the best.
% You can quickly check this via following line, uncomment when you want to run.
% plot( [ wholeData.out ] ) % Plotting idx vs. output value (min distance between the tip of the whip and target);

% For Each wholeData( i ), i = 1:150 has the following values
% [1] tVec     - time vector, Sampling rate was 200Hz (i.e., time step 0.005s).
%                And since the whole simulation time was 2.3second, the wholeData( i ).tVec should be 0 : 0.005 : 2.295
%                The whole number of time steps was 460. (i.e., 460 * 0.005 = 2.3)
% [2] XYZPos   - 87 x 460 element data. 
%                The row values in order, XYZ XYZ XYZ ... XYZ XYZ positions of the following markers...
%                row  1~3  : target XYZ position. The target is stationary so the values doesn't change w.r.t. time. The target position was (X,Y,Z) = (2.395, 0,0)
%                row  4~6  : shoulder XYZ position. Also stationary so the values doesn't change w.r.t. time
%                row  7~9  : elbow XYZ position.
%                row 10~12 : end-effector (handle) XYZ position. The place where the connection happens.
%                row 13~87 : The whip markers, 25-DOF whip model so 25 markers exist in total (i.e., (87-13+1)/3=25)
%                            For image details, please refer to https://dspace.mit.edu/handle/1721.1/127121, figure 3-2 and 3-4
% [3] jointPos - 54 x 460 element data. 
%                The row values in order, are the joint positions (i.e., generalized coordinates) of the whole model
%                in order, [1] shoulder flex/ext
%                          [2] shoulder add/abd
%                          [3] shoulder lateral/medial rotation
%                          [4] elbow    flex/ext
%                          [5~54] X, Y joint angles of each whip segment. 
%                                 The whole whip has 50-DOF. 25 of the 2-DOF sub-models existed. 
%                                 The 2-DOF are 2 rotational joints which are orthogonal with each other, x, and y.
%                                 The data is in X Y X Y X Y ... X Y order. 
%                                 For image details, please refer to https://dspace.mit.edu/handle/1721.1/127121, figure 3-2 and 3-4
% [4] dist        - real-time distance between the tip of the whip and target. This will be useful when you want to "trim" the data. 
% [5] minout      - Optimization value, the minimum distance between the tip of the whip and target.
%                
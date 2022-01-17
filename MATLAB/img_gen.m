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

fig_dir = './myFigures/';

%% ==================================================================
%% (1-) The modal analysis of the whip
%% -- (1A) Calculation of the eigenvalues/eigenvectors, Pole-Zero Plot

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
A = [ zeros( N ), eye( N );
               -inv( M ) * ( K + G ), -inv( M ) * B     ];
           
[eigV, eigD]  = eig( A );                % Calculating the eigenvalues of the statespace matrix

S = [1,zeros( 1, N-1 )]';           

% Input matrix 
% BB = [ zeros( N, 1 ); inv( M )*S ];
% C = ctrbf( A, BB )


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
lwlst = [4.5, 1.5, 1.5];
for i = 1 :  N
    cpx = cpxEig( 2 * i - 1 );
    scatter3( real( cpx ),  imag( cpx ), 0, 400, tmpS{ i },  ...
                    'MarkerFaceColor', c.black, ...
                         'MarkerEdgeColor', c.black, ...
                         'LineWidth', lwlst( i ), 'parent', a )
    scatter3( real( cpx ), -imag( cpx ), 0,400, tmpS{ i },  ...
                     'MarkerFaceColor', c.black, ...
                         'MarkerEdgeColor', c.black, ...
                          'LineWidth', lwlst( i ), 'parent', a )    
                                   

    R( i ) = sqrt( real( cpx ).^2 + imag( cpx ).^2 );  %or whatever radius you want
    x = R(i) * cos(th);
    y = R(i) * sin(th);
    plot(x,y, 'linewidth', 1.3, 'linestyle', '--', 'color', c.black );                       
                    
    text( -R( i )+0.3, -0.8, num2str( -R( i ), '%.2f' ), 'fontsize', 20, 'fontname', 'Myriad Pro', 'color', c.black ) 
    scatter3( -R( i ),     0, 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 2 )
    if i == 3
        continue
    end    
        scatter3(       0, -R(i), 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 2 )
        scatter3(       0,  R(i), 0.1, 100, 'markerfacecolor', c.white, 'markeredgecolor', c.black, 'linewidth', 2 )
    
end

tmpL = 14;
mxAxis = mArrow3( [ -tmpL, 0, -1], [ 2, 0   , -1 ], 'color', c.black, 'stemWidth', 0.05, 'tipWidth', 0.3 );
myAxis = mArrow3( [ 0, -tmpL-1, -1], [ 0, tmpL+1, -1 ], 'color', c.black, 'stemWidth', 0.05, 'tipWidth', 0.3 );

text( 1, -1.5,   'Real', 'fontsize', 30, 'fontname', 'Myriad Pro', 'color', c.black )
text( 0.5, tmpL, 'Imag', 'fontsize', 30, 'fontname', 'Myriad Pro', 'color', c.black )

text( -24, 12, '-6.39     11.26j', 'fontsize', 30, 'fontname', 'Myriad Pro', 'color', c.black )
text( -24, 10, '-0.89     6.910j', 'fontsize', 30, 'fontname', 'Myriad Pro', 'color', c.black )
text( -24, 8,  '-0.03     2.830j', 'fontsize', 30, 'fontname', 'Myriad Pro', 'color', c.black )

scatter3( -25, 12, 0.1, 400, '*', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'linewidth', 4.5 )
scatter3( -25, 10, 0.1, 400, 'd', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'linewidth', 1.5 )
scatter3( -25,  8, 0.1, 400, 'o', 'markerfacecolor', c.black, 'markeredgecolor', c.black, 'linewidth', 1.5 )

tmpL = 17;
set( a, 'ylim', [-tmpL, tmpL] );
set( a, 'xlim', [-tmpL, tmpL] );
axis equal off
grid on

% mySaveFig( gcf, 'S1_PZPlot' )
% exportgraphics( f,'S_fig1.eps')%,'ContentType','vector')

%% -- (1B) EigenMode Plot

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
grid off


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


p1  = plot( xPos_mode1(:,1), zPos_mode1(:,1), '-', 'color', c.black  , 'linewidth', 8 );
p2  = plot( xPos_mode2(:,1), zPos_mode2(:,1), '-', 'color', c.black , 'linewidth', 8 );
p3  = plot( xPos_mode3(:,1), zPos_mode3(:,1), '-', 'color', c.black  , 'linewidth', 8 );

sp1 = scatter( xPos_mode1(:,1), zPos_mode1(:,1), 350, 'o', 'markeredgecolor', c.black , 'markerfacecolor', c.white, 'linewidth', 3 );
sp2 = scatter( xPos_mode2(:,1), zPos_mode2(:,1), 350, 'o', 'markeredgecolor', c.black, 'markerfacecolor', c.white, 'linewidth', 3 );
sp3 = scatter( xPos_mode3(:,1), zPos_mode3(:,1), 350, 'o', 'markeredgecolor', c.black , 'markerfacecolor', c.white, 'linewidth', 3 );

set( gca, 'ylim', [-2,0], 'xlim', [-3.5, 3.5] )
set( gca, 'xticklabel', [], 'yticklabel', [] )
set( gca,'LineWidth',3) 


    % axis box

% exportgraphics( f,'S_fig2.eps')


%% ==================================================================
%% (2-) Plot for iteration vs. Optimization
%% -- (2A) Calling the data + Plot

dir_name  = './myData/optimization_process_3_new/';
N_data    = 6;                                                             % We have 5 targets to analyze.
data_list = cell( 1, N_data );                                             % The whole collection of data     

for i = 1 : N_data
    file_name      = [ dir_name, 'optimization_log_T', num2str( i ), '.txt' ];
    data_list{ i } = myTxtParse( file_name ); 
    
    % Printing out the idx, optimal value output and its input parameter
    opt_val  = min( data_list{ i }.output );
    idx      = find( data_list{ i }.output == opt_val );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
    
end

%% -- (2B) Parsing the data and retriving the best value

% Find and halt if the optimal val has no update 
opt_idx = zeros( 1, 3 );  % The index where the values stop.
tol  = 0.1;               % If the vector norm doesn't change that much, then halt the simulation
ntol = 15;

f = figure( ); a = axes( 'parent', f );
hold on

for i = 1 : 3   % For target 1, 2 and 3

    data     = data_list{ i };
    mov_norm = abs( diff( vecnorm( data.inputPars, 2 ) ) ); 
    
    tmp      = ( mov_norm <= tol ); % Boolean array which shows if mov_norm is within tol 
    cnt      = 0;
    for j = data.Iter  
        if tmp( j )
           cnt = cnt + 1; 
        else
           cnt = 0;
        end
        
        if cnt>= ntol
           opt_idx( i ) = j;
           break 
        end
        
    end

%     plot( data.output( 1: opt_idx( i ) ), 'linewidth', 3 )
    disp( min( data.output( 1 : opt_idx( i ) ) ) )
end

% For targets that is within reach
plot( data_list{ 4 }.Iter, data_list{ 4 }.output, 'linewidth', 3, 'color', [0.4940 0.1840 0.5560] )
plot( data_list{ 5 }.Iter, data_list{ 5 }.output, 'linewidth', 3, 'color', [0.4660 0.6740 0.1880] )
plot( data_list{ 6 }.Iter, data_list{ 6 }.output, 'linewidth', 3, 'color', [0.6350 0.0780 0.1840] )

xlabel( 'Iteration (-)' );  ylabel( '{\it{L^*}}(m)' );
[~, hobj, ~, ~] = legend( 'Target 4', 'Target 5', 'Target 6', 'fontsize', 30 );
ht = findobj(hobj,'type','line');
set(ht,'Linewidth',12);

set( gca, 'xlim', [1, 80] )
set( gca, 'ylim', [0, 3.5] )

% For saving the figure of the iteration
exportgraphics( f, [ fig_dir,'fig2.eps'],'ContentType','vector')
exportgraphics( f, [ fig_dir,'fig2.pdf'] )


% Displaying the best values within the opt_idx 
for i = 1 : 3 
    opt_val  = min( data_list{ i }.output( 1 : opt_idx( i ) ) );
    idx      = find( opt_val == data_list{ i }.output( 1 : opt_idx( i ) ) );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
end


%% -- (2C) Comparing with/without gravity compensation
dir_name  = './myData/optimization_process_3_new/';
data_list = cell( 1, 2 );                                          
name      = {'optimization_log_T1', 'optimization_log_T1_wo_g'};

for i = 1 : 2
    file_name      = [ dir_name, name{ i }, '.txt' ];
    data_list{ i } = myTxtParse( file_name ); 
    
    % Printing out the idx, optimal value output and its input parameter
    opt_val  = min( data_list{ i }.output );
    idx      = find( data_list{ i }.output == opt_val );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
    
end

% Find and halt if the optimal val has no update 
opt_idx = zeros( 1, 2 );  % The index where the values stop.
tol  = 0.1;               % If the vector norm doesn't change that much, then halt the simulation
ntol = 15;

f = figure( ); a = axes( 'parent', f );
hold on

for i = 1 : 2   % For target 1, 2 and 3

    data     = data_list{ i };
    mov_norm = abs( diff( vecnorm( data.inputPars, 2 ) ) ); 
    
    tmp      = ( mov_norm <= tol ); % Boolean array which shows if mov_norm is within tol 
    cnt      = 0;
    for j = data.Iter  
        if tmp( j )
           cnt = cnt + 1; 
        else
           cnt = 0;
        end
        
        if cnt>= ntol
           opt_idx( i ) = j;
           break 
        end
        
    end

    plot( data.output( 1: opt_idx( i ) ), 'linewidth', 3 )
    disp( min( data.output( 1 : opt_idx( i ) ) ) )
end


for i = 1 : 2 
    opt_val  = min( data_list{ i }.output( 1 : opt_idx( i ) ) );
    idx      = find( opt_val == data_list{ i }.output( 1 : opt_idx( i ) ) );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
end
% ==================================================================
%% (3-) Miscellaneous Plots
%% -- (3A) Calling the data + Plot

for i = 1 : 6
    
   rawData{ i } = myTxtParse( ['myData/simulation_log_3/data_log_T', num2str( i ), '.txt']  );
%     rawData{ i } = myTxtParse( ['data_log_dense_T', num2str( i ), '.txt']  );
   
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );
   
end

%% -- (3B) Plot of Movement Snapshots

idx = 6;        % Choose Target Type

idxS = find( rawData{ idx }.output == min( rawData{ idx }.output ), 1, 'first'  );

tIdx = [1, 68, idxS;
        1, 37, idxS; 
        1, 24, idxS;
        1, 37, idxS; 
        1, 37, idxS;
        1, 24, idxS ];

% viewArr = [ 49.9456, 4.7355;
%             68.8342, 6.0279;
%             44.3530, 7.4481];    

viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;            
            49.9456, 4.7355;
            49.9456, 4.7355];

alpha = [0.3, 0.5, 1.0];                                              % The alpha values of each screen shot   
f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis square; hold on;


color_arr = [  0.4940, 0.1840, 0.5560; ...
              0.4660, 0.6740, 0.1880; ...
              0.6350 0.0780 0.1840; ...
              0, 0.4470, 0.7410; ...
              0.8500, 0.3250, 0.0980; ...
              0.9290, 0.6940, 0.1250 ];
          
cTarget = color_arr( idx, : );

mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 500, ...        % Setting the handle of the ZFT Plot, 
                   'parent', a,   'LineWidth', 1,               ...       % For the main plot (s1)
                   'MarkerFaceColor', cTarget, 'MarkerEdgeColor', cTarget, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );

      
for i = 1 : 3
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ 0.2,0.2, 0.2, alpha( i ) ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), 800, ... 
                   'parent', a,   'LineWidth',  5, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i )  );

               
%     p3 = plot3(  rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), ...
%                  'parent', a, ...
%                 'linewidth', 8, 'color', [ c.purple_plum, alpha( i ) ] );
            
    p4 = scatter3( rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), 100, ... 
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor',  [0.75, 0, 0.75], ...[0.75, 0, 0.75], ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );
               
               
end    


tmpLim = 2.4;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : ) )  %  [BACKUP] [Target #1] 16.3213    6.0865
      

set( a, 'xtick', [-2, 0, 2] ); set( a, 'xticklabel', ["-2", "\fontsize{50}X (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-2, 0, 2] ); set( a, 'yticklabel', ["-2", "\fontsize{50}Y (m)", "+2"] ); % ["-2", "Y[m]", "+2"] )
set( a, 'ztick', [-2, 0, 2] ); set( a, 'zticklabel', ["-2", "\fontsize{50}Z (m)", "+2"] ); % ["-2", "Z[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 )
exportgraphics( f,[fig_dir, 'F3_',num2str(idx),'a_timelapse.pdf'],'ContentType','vector')


% mySaveFig( f, ['output', num2str( idx )] );

%% -- (3C) The end-effector and the elbow's trajectory 

% Plotting the ``trace'' or ``path'' of the upper-limb movement.
idx = 6;

tStarts = [0,0 ,0,0,0, 0];
tEnds   = [0.95 ,0.95    ,0.58333,0.95   , 0.95   , 0.58333];

tStart = tStarts( idx );
tEnd  = tEnds( idx );


idxS = find( rawData{ idx }.currentTime >= tStart & rawData{ idx }.currentTime <= tEnd );	
% idxS = idxS( 1 : 3 : end);
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;

switch idx 
   
    case 4
        idx_list = [1, 25, 35, 43, 57];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 5
        idx_list = [1, 25, 33, 42, 57];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 6
        idx_list = [1,  19, 25, 35];
        alpha    = [0.2, 0.4, 0.7, 1.0];
        
    case 1
        idx_list = [1, 30, 38, 57];
        alpha    = [0.2, 0.4, 0.7, 1.0];
    case 2
        idx_list = [1,  22, 28, 35];
        alpha    = [0.2, 0.4, 0.7, 1.0];        
    case 3
        idx_list = [1,  22, 28, 35];
        alpha    = [0.2, 0.4, 0.7, 1.0];        
end


scatter3( rawData{ idx }.geomXPositions( 2, idxStart ), ...
          rawData{ idx }.geomYPositions( 2, idxStart ), ...
          rawData{ idx }.geomZPositions( 2, idxStart ), 2000, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

plot3( rawData{ idx }.geomXPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 3, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 3, 'color', c.black );

plot3( rawData{ idx }.geomXPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 4, idxStart : idxEnd ), ...
      'parent', a,   'LineWidth', 3, 'color', c.black );



itmp = 1;
for i = idx_list
    
    plot3( rawData{ idx }.geomXPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomYPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomZPositions( 2:4, idxS( i ) ), ...
                          'parent', a,   'LineWidth', 7, 'color', [ c.black, alpha( itmp ) ]  )    
%     
    scatter3( rawData{ idx }.geomXPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 3, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 6, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black , ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha( itmp ) );           
        
           
    scatter3( rawData{ idx }.geomXPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 4, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 6, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black,  ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha( itmp ) );           
    itmp = itmp + 1;
end


viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;            
            49.9456, 4.7355;
            49.9456, 4.7355];

tmpLim = 0.6;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : )   )  
      
      
set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["-0.6", "\fontsize{50}X (m)", "+0.6"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["-0.6", "\fontsize{50}Y (m)", "+0.6"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["-0.6", "\fontsize{50}Z (m)", "+0.6"] ); % ["-2", "X[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )
exportgraphics( f,[fig_dir, 'F3_',num2str(idx),'b_timelapse_EL_EE.pdf'],'ContentType','vector');

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
% exportgraphics( f,['F3_',num2str(idx),'b_timelapse_EL_EE.eps'] ) %'ContentType','vector')

%% -- (3D) Best-fit-Plane Identification/Calculation


% Plotting the ``trace'' or ``path'' of the upper-limb movement.
idx  = 6;

color_arr = [  0.4940, 0.1840, 0.5560; ...
              0.4660, 0.6740, 0.1880; ...
              0.6350 0.0780 0.1840; ...
              0, 0.4470, 0.7410; ...
              0.8500, 0.3250, 0.0980; ...
              0.9290, 0.6940, 0.1250 ];
          
t_start   = [     0,     0,     0,     0,     0,    0 ];
t_end     = [ 0.950, 0.950, 0.583, 0.950, 0.950, 0.583];


% Find the start and end point of the data
idxS = find( rawData{ idx }.currentTime >= t_start( idx ) & rawData{ idx }.currentTime <= t_end( idx ) );	
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;

% The end-effector index is the 4th one, 
% 1st one is target, 2nd one is shoulder, 3rd one is elbow and 4th the end-effector
x = rawData{ idx }.geomXPositions( 4, idxS )';
y = rawData{ idx }.geomYPositions( 4, idxS )';
z = rawData{ idx }.geomZPositions( 4, idxS )';


p  = [ x, y, z ];                                                          % The position of the end-effector, N x 3
pC = mean( p );                                                            % The mean of the data
pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig( pn' * pn );

% The eigenvalues are ordered from low to high
% Getting the first eigenvectors and find the vectors that are orthogonal to it.

w = null( eigvecs( : , 1)' ); 
pC
eigvecs( :, 1 )


scatter3( rawData{ idx }.geomXPositions( 2, idxStart ), ...
          rawData{ idx }.geomYPositions( 2, idxStart ), ...
          rawData{ idx }.geomZPositions( 2, idxStart ), 300, 's', ... 
          'parent', a,   'LineWidth', 4, ...
          'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
          'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

plot3( rawData{ idx }.geomXPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 4, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 4, 'color', [ color_arr( idx, : ), 0.3] )

scatter3(  rawData{ idx }.geomXPositions( 4, idxS ), ...
           rawData{ idx }.geomYPositions( 4, idxS ), ...
           rawData{ idx }.geomZPositions( 4, idxS ), 200, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_arr( idx, : ), ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           
       

tmpLim = 0.55;      

tmp = 0;
for i = 1 : length( idxS )  % Brute force calculation of the distance.
%     ttmp = ( eigvecs(1,1) * ( x(i) - pC(1) ) ) + ( eigvecs(2,1) * ( y(i) - pC(2) ) )^2 + ( eigvecs(3,1) * ( z(i) - pC(3) )  )^2 ;
    ttmp = abs( ( eigvecs(1,1) * ( x( i ) - pC( 1 ) ) ) + ...
                ( eigvecs(2,1) * ( y( i ) - pC( 2 ) ) ) + ...
                ( eigvecs(3,1) * ( z( i ) - pC( 3 ) ) ) );
    tmp = tmp + ttmp^2;
end

sqrt( tmp/length( idxS ) )

% [P,Q] = meshgrid( -tmpLim: 0.1 : tmpLim );                       
if idx == 4
    [P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          
elseif idx == 5     
    [P,Q] = meshgrid( -tmpLim + 0.05: 0.1 : tmpLim + 0.05);                      
elseif idx == 3 
    [P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.2);          
elseif idx == 6
    [P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.2);              
elseif idx == 1 
    [P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          
else
    [P,Q] = meshgrid( -tmpLim: 0.1 : tmpLim);                      
end

XX = pC( 1 ) + w( 1, 1 ) * P + w( 1, 2 ) * Q;                              
YY = pC( 2 ) + w( 2, 1 ) * P + w( 2, 2 ) * Q;                              
ZZ = pC( 3 ) + w( 3, 1 ) * P + w( 3, 2 ) * Q;


scatter3(  pC( 1 ) , pC( 2 ), pC( 3 ), 400, 'd',... 
           'parent', a,   'LineWidth', 6, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_arr( idx, : ), ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           

if idx == 1 || idx == 2 || idx == 4
    mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', color_arr( idx, : ), 'tipWidth', 0.03, 'stemWidth', 0.008 );
else
    mArrow3( pC, pC + 0.3 * eigvecs( : , 1 )', 'color', color_arr( idx, : ), 'tipWidth', 0.03, 'stemWidth', 0.008 );
end
% mArrow3( pC, pC + 0.3 * eigvecs( : , 2 )', 'colorcl', c.green, 'tipWidth', 0.01, 'stemWidth', 0.004 )
% mArrow3( pC, pC + 0.3 * eigvecs( : , 3 )', 'color', c.green, 'tipWidth', 0.01, 'stemWidth', 0.004 )

surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', color_arr( idx, : ), 'facealpha', 0.3 );
tmpLim2 = 0.7;

viewArr = [97.3451, 5.0653;
          142.4901, 3.2252;
          133.9720, 3.2252;
          97.3451, 5.0653;
          142.4901, 3.2252;
          133.9720, 3.2252;];

set( a,   'XLim',   [ - tmpLim2, tmpLim2 ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim2, tmpLim2 ] , ...    
          'ZLim',   [ - tmpLim2, tmpLim2 ] , ...
          'view',   viewArr( idx, : ) )  

if idx == 1 || idx == 4
    set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["", "\fontsize{43}X (m)", ""] )
    set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["\fontsize{35}-0.5", "\fontsize{43}Y (m)", "\fontsize{35}+0.5"] ); % ["-2", "X[m]", "+2"] )
    set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["\fontsize{35}-0.5", "\fontsize{43}Z (m)", "\fontsize{35}+0.5"] ); % ["-2", "X[m]", "+2"] )        
    set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["", "\fontsize{43}X (m)", ""] )
    set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["\fontsize{35}-0.5", "\fontsize{43}Y (m)", "\fontsize{35}+0.5"] ); % ["-2", "X[m]", "+2"] )
    set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["\fontsize{35}-0.5", "\fontsize{43}Z (m)", "\fontsize{35}+0.5"] ); % ["-2", "X[m]", "+2"] )            
else
    set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["-0.5", "\fontsize{50}X (m)", "+0.5"] )
    if idx == 2 || idx == 5
        set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["", "", ""] ); % ["-2", "X[m]", "+2"] )
        annotation('textbox', [0.30, 0.13, 0.1, 0.], 'EdgeColor','none','string', "\fontsize{50}Y (m)")
        annotation('textbox', [0.23, 0.16, 0.1, 0.], 'EdgeColor','none','string', "\fontsize{40}-0.5")
        annotation('textbox', [0.41, 0.11, 0.1, 0.], 'EdgeColor','none','string', "\fontsize{40}+0.5")
    else
        set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["-0.5", "\fontsize{50}Y (m)", "+0.5"] ); % ["-2", "X[m]", "+2"] )
    end
    set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["-0.5", "\fontsize{50}Z (m)", "+0.5"] ); % ["-2", "X[m]", "+2"] )    
    
end


set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )

exportgraphics( f,[fig_dir, 'F4_',num2str(idx),'a_best_fit_plane.pdf'],'ContentType','vector' )

%% -- (3E) Contribution of each Movements


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

idx = 6;

t_start   = [     0,     0,     0,     0,     0,    0 ];
t_end     = [ 0.950, 0.950, 0.583, 0.950, 0.950, 0.583];

idxS = find( rawData{ idx }.currentTime >= t_start( idx ) & rawData{ idx }.currentTime <= t_end( idx ) );	
idxStart = min( idxS ); idxEnd = max( idxS );

clear c1 c2 c3 c4

    
dp = rawData{ idx }.qPos( 1:4, : ) - rawData{ idx }.qPos0(2:end, : );
% dv =   rawData{ idx }.jointVelActual( 1:4, : ) - rawData{ idx }.vZFT;

c1_K = dp' * v1;
c2_K = dp' * v2;
c3_K = dp' * v3;
c4_K = dp' * v4;

% c1_B = dv' * v1;
% c2_B = dv' * v2;
% c3_B = dv' * v3;
% c4_B = dv' * v4;

norm( c1_K( idxS ), 2 )
norm( c2_K( idxS ), 2 )
norm( c3_K( idxS ), 2 )
norm( c4_K( idxS ), 2 )


f = figure( ); a = axes( 'parent', f );hold on;

plot( rawData{idx}.currentTime( idxS ) - t_start( idx ), c1_K( idxS ), 'linewidth', 4, 'linestyle', '-'  , 'color', c.black )
plot( rawData{idx}.currentTime( idxS ) - t_start( idx ), c2_K( idxS ), 'linewidth', 4, 'linestyle', '--' , 'color', c.black )
plot( rawData{idx}.currentTime( idxS ) - t_start( idx ), c3_K( idxS ), 'linewidth', 4, 'linestyle', ':'  , 'color', c.black )
plot( rawData{idx}.currentTime( idxS ) - t_start( idx ), c4_K( idxS ), 'linewidth', 4, 'linestyle', '-.' , 'color', c.black )

[hleg1, hobj1] = legend( "w_1","w_2","w_3","w_4", 'fontsize', 50, 'location', 'northwest', 'fontname', 'myriad pro' );
   set(hleg1,'position',[0.15 0.70 0.1 0.15])


set( a,   'XLim',   [ 0, rawData{idx}.currentTime( idxEnd ) - t_start( idx ) ], 'fontsize', 35,'fontname', 'myriad pro' )
set( a,   'YLim',   [ -1, 1]                                          , 'fontsize', 35,'fontname', 'myriad pro' )
set( a, 'ytick', [-1, -0.5, 0, 0.5, 1], 'yticklabel', ["-1", "", "0", "", "1"], 'fontname','myriad pro'  )

if idx == 1
    set( a, 'xtick', [0, 0.3, 0.6, 0.9], 'xticklabel', ["0", "0.3", "0.6", "0.9"], 'fontname','myriad pro', 'fontsize', 40  )
elseif idx  ==2 
    set( a, 'xtick', [0, 0.3, 0.6, 0.9], 'xticklabel', ["0", "0.3", "0.6", "0.9"], 'fontname','myriad pro', 'fontsize', 40  )
        
elseif idx ==3 
    set( a, 'xtick', [0, 0.25, 0.5], 'xticklabel', ["0", "0.25", "0.5" ], 'fontname','myriad pro', 'fontsize', 40  )

elseif idx  == 4
    set( a, 'xtick', [0, 0.3, 0.6, 0.9], 'xticklabel', ["0", "0.3", "0.6", "0.9"], 'fontname','myriad pro', 'fontsize', 40  )

elseif idx  == 5
    set( a, 'xtick', [0, 0.3, 0.6, 0.9], 'xticklabel', ["0", "0.3", "0.6", "0.9"], 'fontname','myriad pro', 'fontsize', 40  )

elseif idx == 6
    set( a, 'xtick', [0, 0.25, 0.5], 'xticklabel', ["0", "0.25", "0.5" ], 'fontname','myriad pro', 'fontsize', 40  )

    
end

% xlabel( 'Time (sec)'      , 'fontsize', 50, 'fontname', 'myriad pro' ); 
% ylabel( 'Coefficients (-)', 'fontsize', 50, 'fontname', 'myriad pro' );


exportgraphics( f,[ fig_dir, 'F5_',num2str(idx),'_coefficients.pdf'],'ContentType','vector')


%% ==================================================================
%% (4-) Plots for Position Controller
%% -- (4A) Optimization Result

N_data    = 6;                                                             % We have 5 targets to analyze.
data_list = cell( 1, N_data );                                             % The whole collection of data     

for i = 1 : N_data
    file_name      = ['myData/position_controller_2/optimization_result/optimization_log_T', num2str( i ), '.txt'];
%     file_name =  ['myData/optimization_process_3_new/optimization_log_T1_w_tapered.txt']
    data_list{ i } = myTxtParse( file_name ); 
    
    % Printing out the idx, optimal value output and its input parameter
%     opt_val  = min( data_list{ i }.output );
%     idx      = find( data_list{ i }.output == opt_val );
%     mov_pars = data_list{ i }.inputPars( :, idx )';
%     
%     fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
%     fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
    
end

% Find and halt if the optimal val has no update 
opt_idx = zeros( 1, 6 );  % The index where the values stop.
tol  = 0.1;               % If the vector norm doesn't change that much, then halt the simulation
ntol = 15;

f = figure( ); a = axes( 'parent', f );
hold on

for i = 1 : 6   % For target 1, 2 and 3

    data     = data_list{ i };
    mov_norm = abs( diff( vecnorm( data.inputPars, 2 ) ) ); 
    
    tmp      = ( mov_norm <= tol ); % Boolean array which shows if mov_norm is within tol 
    cnt      = 0;
    for j = data.Iter  
        if tmp( j )
           cnt = cnt + 1; 
        else
           cnt = 0;
        end
        
        if cnt>= ntol
           opt_idx( i ) = j;
           break 
        end
        
    end

    plot( data.output( 1: opt_idx( i ) ), 'linewidth', 3 )
    disp( min( data.output( 1 : opt_idx( i ) ) ) )
end

for i = 4: 6
    data_list{ i }.opt_idx = find( data_list{ i }.output( 1 : opt_idx( i ) ) == 0.1 );
    mean( data_list{ i }.inputPars( :, data_list{ i }.opt_idx )' )
end

% For targets that is within reach
plot( data_list{ 4 }.Iter, data_list{ 4 }.output, 'linewidth', 3 )
plot( data_list{ 5 }.Iter, data_list{ 5 }.output, 'linewidth', 3 )

xlabel( 'Iteration (-)' );  ylabel( '{\it{L^*}}(m)' );
[~, hobj, ~, ~] = legend( 'Target 1', 'Target 2', 'Target 3', 'Target 4', 'Target 5', 'fontsize', 30 );
ht = findobj(hobj,'type','line');
set(ht,'Linewidth',12);

set( gca, 'xlim', [1, max( opt_idx ) ] )
set( gca, 'ylim', [0, 3.5] )

% For saving the figure of the iteration
% exportgraphics( f, [ fig_dir,'S_fig2.eps'],'ContentType','vector')
% exportgraphics( f, [ fig_dir,'S_fig2.pdf'] )


% Displaying the best values within the opt_idx 
for i = 1 : 6
    opt_val  = min( data_list{ i }.output( 1 : opt_idx( i ) ) );
    idx      = find( opt_val == data_list{ i }.output( 1 : opt_idx( i ) ), 1, 'first' );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
end


%% -- (4B) Calling the data + Plot

for i = 1 : 6
    
   rawData{ i } = myTxtParse( ['myData/position_controller_2/data_log/data_log_T', num2str( i ), '.txt']  );
%     rawData{ i } = myTxtParse( ['data_log_dense_T', num2str( i ), '.txt']  );
   
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );
   
end

%% -- (4C) Plot of Movement Snapshots

idx = 3;        % Choose Target Type

idxS = find( rawData{ idx }.output == min( rawData{ idx }.output )  );

tIdx = [1, 33, idxS;
        1, 37, idxS; 
        1, 20, idxS;
        1, 33, idxS;
        1, 37, idxS; 
        1, 20, idxS];

viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355];

color_arr = [ 0.4940, 0.1840, 0.5560; ...
              0.4660, 0.6740, 0.1880; ...
              0.6350, 0.0780, 0.1840; ...
                   0, 0.4470, 0.7410; ...
              0.8500, 0.3250, 0.0980; ...
              0.9290, 0.6940, 0.1250 ];
        
cTarget = color_arr( idx, : );        
        
alpha = [0.2, 0.5, 1.0];
% alpha = [0.4, 0.6, 1.0];                                              % The alpha values of each screen shot   
f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis square; hold on;


mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 500, ...        % Setting the handle of the ZFT Plot, 
                   'parent', a,   'LineWidth', 1,               ...       % For the main plot (s1)
                   'MarkerFaceColor', cTarget, 'MarkerEdgeColor', cTarget, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );

      
for i = 1 : 3
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ c.black, alpha( i ) ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), 800, ... 
                   'parent', a,   'LineWidth',  5, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
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
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor',  [0.75, 0, 0.75], ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );
               
               
end    


tmpLim = 2.4;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : ) )  %  [BACKUP] [Target #1] 16.3213    6.0865
      


set( a, 'xtick', [-2, 0, 2] ); set( a, 'xticklabel', ["-2", "\fontsize{50}X (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-2, 0, 2] ); set( a, 'yticklabel', ["-2", "\fontsize{50}Y (m)", "+2"] ); % ["-2", "Y[m]", "+2"] )
set( a, 'ztick', [-2, 0, 2] ); set( a, 'zticklabel', ["-2", "\fontsize{50}Z (m)", "+2"] ); % ["-2", "Z[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 )

exportgraphics( f,[fig_dir, 'S3_',num2str(idx),'a_posctrl_timelapse.pdf'],'ContentType','vector')


% mySaveFig( f, ['output', num2str( idx )] );

%% -- (4C) The end-effector and the elbow's trajectory 

% Plotting the ``trace'' or ``path'' of the upper-limb movement.
idx = 6;

tStarts = [0,0 ,0,0,0, 0];
tEnds   = [0.95, 0.95, 0.583, 0.95,  0.95, 0.583];

tStart = tStarts( idx );
tEnd  = tEnds( idx );

idxS = find( rawData{ idx }.currentTime >= tStart & rawData{ idx }.currentTime <= tEnd );	
% idxS = idxS( 1 : 3 : end);
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis equal; hold on;
scatter3( rawData{ idx }.geomXPositions( 2, idxStart ), ...
          rawData{ idx }.geomYPositions( 2, idxStart ), ...
          rawData{ idx }.geomZPositions( 2, idxStart ), 2000, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

plot3( rawData{ idx }.geomXPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 3, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 3, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 3, 'color', c.black );

plot3( rawData{ idx }.geomXPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomYPositions( 4, idxStart : idxEnd ), ...
       rawData{ idx }.geomZPositions( 4, idxStart : idxEnd ), ...
      'parent', a,   'LineWidth', 3, 'color', c.black );


switch idx 
   
    case 1
        idx_list = [5, 20, 28, 38, 50];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 2
        idx_list = [1, 20, 28, 37, 55];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 3
        idx_list = [2, 16, 22,35];
        alpha    = [0.3, 0.6, 0.8, 1.0];
    case 4
        idx_list = [5, 20, 28, 38, 50];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 5
        idx_list = [1, 20, 28, 37, 55];
        alpha    = [0.2, 0.4, 0.6, 0.8, 1.0];
    case 6
        idx_list = [2, 16, 22,35];
        alpha    = [0.3, 0.6, 0.8, 1.0];
end

itmp = 1;
for i = idx_list
    
    plot3( rawData{ idx }.geomXPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomYPositions( 2:4, idxS( i ) ), ...
           rawData{ idx }.geomZPositions( 2:4, idxS( i ) ), ...
                          'parent', a,   'LineWidth', 7, 'color', [ c.black, alpha( itmp ) ]  )    
%     
    scatter3( rawData{ idx }.geomXPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 3, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 3, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 6, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black , ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha( itmp ) );           
        
           
    scatter3( rawData{ idx }.geomXPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomYPositions( 4, idxS( i ) ), ...
              rawData{ idx }.geomZPositions( 4, idxS( i ) ), 2000, ... 
               'parent', a,   'LineWidth', 6, ...
               'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black,  ...
               'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha( itmp ) );           
    itmp = itmp + 1;
end


viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355;
            49.9456, 4.7355 ];

tmpLim = 0.6;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : )   )  
      
      
set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["-2", "\fontsize{50}X (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-0.5, 0, 0.5] ); set( a, 'yticklabel', ["-2", "\fontsize{50}Y (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["-2", "\fontsize{50}Z (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )

% mySaveFig( f, ['output', num2str( idx )] );
exportgraphics( f,[fig_dir, 'S3_',num2str(idx),'b_timelapse_posctrl_EL_EE.pdf'],'ContentType','vector')

%% ==================================================================
%% (5-) Cover Image
%% -- (5A) Read data
for i = 1 : 3
    
   rawData{ i } = myTxtParse( ['myData/simulation_log/data_log_long_T', num2str( i ), '.txt']  );
%     rawData{ i } = myTxtParse( ['data_log_dense_T', num2str( i ), '.txt']  );
   
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );
   
end

%% -- (5B) Color Gradiation
close all

idx = 3;

N = 25;


f = figure( ); a = axes( 'parent', f ); hold on
axis equal; 
cArr = flipud( colormap( bone( 3 * N ) ) );


switch idx 
    case 1
        cTarget = [1,0,0];
    case 2
        cTarget = [0,0.5,0];
    case 3
        cTarget = [1,0,0];
end

viewArr = [ 10, 10;
            30, 4.7;
            49.9456, 4.7355 ];

alpha = [0.2, 0.5, 1.0];                                                   % The alpha values of each screen shot   

idxS = find( rawData{ idx }.outputVal == min( rawData{ idx }.outputVal )  );

tIdx = [50, 380, idxS;
        50, 5*37, idxS; 
        50, 5*61, idxS];        


mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 500, ...        % Setting the handle of the ZFT Plot, 
                   'parent', a,   'LineWidth', 1,               ...       % For the main plot (s1)
                   'MarkerFaceColor', cTarget, 'MarkerEdgeColor', cTarget, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );


for i = 3: 28
   plot3( rawData{ idx }.geomXPositions( i + 1, 50 : end ), ...
          rawData{ idx }.geomYPositions( i + 1, 50 : end ), ...
          rawData{ idx }.geomZPositions( i + 1, 50 : end ), ...
           'parent', a, 'color', cArr( i , : ), 'linewidth', 2  )    

    
    
end


for i = 1 : 3
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ c.black, alpha( i ) ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), 800, ... 
                   'parent', a,   'LineWidth', 5, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );

               
%     p3 = plot3(  rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), ...
%                  'parent', a, ...
%                 'linewidth', 8, 'color', [ c.purple_plum, alpha( i ) ] );
            
    p4 = scatter3( rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), 200, ... 
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0, 0.4470, 0.7410], ... c.purple_plum, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );
               
               
end    
        
       

tmpLim = 2.5;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : )   )  
set( a, 'xtick', [-1, 0, 1] ); set( a, 'xticklabel', [] )
set( a, 'ytick', [-1, 0, 1] ); set( a, 'yticklabel', [] )
set( a, 'ztick', [-1, 0, 1] ); set( a, 'zticklabel', [] )
set(a,'LineWidth',3.5 ); set(a, 'TickLength',[0.000 0.000]);

axis equal

exportgraphics( f,['CL_',num2str(idx),'_gradient.pdf'],'ContentType','vector')

%% ==================================================================
%% (6-) Cover Image
%% -- (6A) Force-end-effector Plot
close all

idx  = 3;

data = myTxtParse( ['./myData/force_data/data_log_T', num2str( idx ), '_Force.txt' ] );

%%
D = [0.95, 0.579, 0.95];

[ ~,closestIndex] = min(abs( data.currentTime - ( 0.1 + D( idx ) ) ));

myC = [c.pink; c.blue; c.green ];

ttmp = closestIndex;
% ttmp = find( data.minVal == min( data.minVal ) );

EEpos = data.geomXYZPositions(  10:12, 1:ttmp );
EEvel = data.geomXYZVelocities( 10:12, 1:ttmp );

fVec  = data.forceVec( :, 1:ttmp + 20 );

plot3( EEpos( 1,: ), EEpos( 2,: ), EEpos( 3,: ), 'color', myC( idx, : ), 'linewidth', 5 );
hold on 
scatter3( EEpos( 1,: ), EEpos( 2,: ), EEpos( 3,: ), 400, 'o', 'markeredgecolor', myC( idx, : ), 'markerfacecolor', c.white, 'markerfacealpha', 0.8, 'linewidth', 6 );

% Adding force vector.
scale = 0.02;
quiver3(  data.geomXYZPositions( 10,1:ttmp  ), data.geomXYZPositions( 11, 1:ttmp  ), data.geomXYZPositions( 12,1:ttmp  ),...
          data.forceVec( 1, 1:ttmp ) * scale, data.forceVec( 2, 1:ttmp  ) * scale, data.forceVec( 3, 1:ttmp ) * scale, ...
          'linewidth', 5, 'color', c.black, 'AutoScale', 'off'  )

   
      
axis equal


% for i = 1: ttmp
%    J_vals( :, :, i ) = J_func( L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
%    ZFTvel( :, i ) = J_vals( :, :, i ) * data.jointVelActual( 1:4, i );
%    
%    Cx = Cx_func(  L1, L2, qMat( i, 1 ), qMat( i, 2 ), qMat( i, 3 ), qMat( i, 4 ) );
%    Kx = Cx^-1;       
%    
%    tmp1( :, i ) = Kx * ( pEE_ZFT( :, i ) - EEpos( :, i ) );
% end

% What is the force due to Kx x

% tmp2 = 0.05 * Kx * ( ZFTvel - EEvel );

% tmp = tmp1 + tmp2;
% tmp = tmp1;

% scale = 0.0000001;
% quiver3(  EEpos( 1,1:ttmp ), EEpos( 2, 1:ttmp ), EEpos( 3,1:ttmp ),...
%           tmp( 1,1:ttmp) * scale, tmp( 2, 1:ttmp) * scale, tmp( 3, 1:ttmp ) * scale, ...
%           'linewidth', 5, 'color', c.green, 'AutoScale', 'on'  )

tmpLim = 0.8;
set( gca, 'XLim',   [ -tmpLim , tmpLim ] , ...                  
                     'YLim',   [ -tmpLim , tmpLim ] , ...    
                     'ZLim',   [ -tmpLim , tmpLim ] , ...
                     'view',   [0   0]     )           
                 
set( gca, 'xtick', [-0.5, 0, 0.5] ); set( gca, 'xticklabel', ["-0.5", "\fontsize{50}X (m)", "+0.5"] ); % ["-2", "X[m]", "+2"] )
set( gca, 'ytick', [-0.5, 0, 0.5] ); set( gca, 'yticklabel', ["-0.5", "\fontsize{50}Y (m)", "+0.5"] ); % ["-2", "X[m]", "+2"] )
set( gca, 'ztick', [-0.5, 0, 0.5] ); set( gca, 'zticklabel', ["-0.5", "\fontsize{50}Z (m)", "+0.5"] ); % ["-2", "X[m]", "+2"] )
set(gca,'LineWidth',3.0 ); set(gca, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )
                 

%% -- (6B) Time-Power Plot

f = figure( ) ;
a = axes( 'parent', f ) ;

ttmp = find( data.minVal == min( data.minVal ) );
st = 20;

pIn = sum( data.forceVec .* data.geomXYZVelocities( 10:12, : ));
hold on

D = [0.95, 0.579, 0.95];

yyaxis right
ylabel('Distance (m)')
plot( a, data.currentTime( 1: ttmp + st ) , data.minVal( 1: ttmp + st) )

yyaxis left
ylabel('Power-In (W)')
plot( a, data.currentTime( 1: ttmp + st ) , pIn( 1: ttmp + st) )
set( gca, 'xtick', [0.1, 0.1+D( idx ), round( data.currentTime( ttmp ), 2 ) ] )
set( gca, 'xlim', [0, max( data.currentTime( 1: ttmp + st ) ) ] )

xlabel( 'Time (sec)' ); 
 mySaveFig( f, ['output', num2str( idx )] );


%% ==================================================================
%% (7-) Robustness Analysis
%% -- (7A) Data read

dir_name   = './myData/robustness_analysis/';

% file_names = { 'out1_init.txt', 'out1_final.txt', 'out1_D.txt' };
% file_names = { 'out4_init.txt', 'out4_final.txt', 'out4_D.txt' };
% file_names = { 'out5_init.txt', 'out5_final.txt', 'out5_D.txt' };
file_names = { 'out6_init.txt', 'out6_final.txt', 'out6_D.txt' };
data_list  = cell( 1, 3 );
mu         = zeros( 1, 3 );
sigma      = zeros( 1, 3 );


f = figure( ); a = axes( 'parent', f );
hold on
for i = 1 : 3
    data_list{ i } = myTxtParse( [ dir_name, file_names{ i } ] );
%        mu( i ) = mean( data_list{ i }.output );
%     sigma( i ) =  std( data_list{ i }.output );
    
end
hit_rate1 = length( find( data_list{1}.output == 0.1 )  )/200 * 100
hit_rate2 = length( find( data_list{2}.output == 0.1 )  )/200 * 100
hit_rate3 = length( find( data_list{3}.output == 0.1 )  )/200 * 100

plot( 1 * ones( 1, 200 ), data_list{ 1 }.output, 'o', 'markeredgecolor', 'k' )
plot( 2 * ones( 1, 200 ), data_list{ 2 }.output, 'o', 'markeredgecolor', 'k' )
plot( 3 * ones( 1, 200 ), data_list{ 3 }.output, 'o', 'markeredgecolor', 'k' )

% errorbar( [1,2,3], mu, sigma, '.', 'linewidth', 4, 'CapSize',60, 'markersize', 100, 'Color', 'k', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'k')

yline( 0.05071, 'linewidth', 4, 'linestyle', '--' )
set( a, 'xtick', [1,2,3], 'xticklabel', {'Case A', 'Case B', 'Case C'}, 'xlim', [0, 4] )%, 'ylim', [0, 0.40] )
ylabel( a, '{\it L^{*}} (m)' )

exportgraphics( f, [ fig_dir,'S_fig4.pdf'] )

%% ==================================================================
%% (8-) Gravity Compensation Analysis
%% -- (8A) Data read

dir_name   = './myData/with_without_grav/data_log/';
file_names = { 'data_log_w_g.txt', 'data_log_wo_g.txt' };

N = length( file_names );
data_list  = cell( 1, N );
for i = 1 : N
    rawData{ i } = myTxtParse( [ dir_name, file_names{ i } ] );
    
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );    
end


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

%% -- (8B) Plot Comparison 2D

f = figure( ); a = axes( 'parent', f );
idx = 1;

tStarts = [0, 0];
tEnds   = [0.94547, 0.89115];

color_arr = [      0, 0.4470, 0.7410; ...
      0.8500, 0.3250, 0.0980; ...
      0.9290, 0.6940, 0.1250; ...
      0.4940, 0.1840, 0.5560; ...
      0.4660, 0.6740, 0.1880 ];

tStart = tStarts( idx );
tEnd   =   tEnds( idx );

idxS1 = find( rawData{ 1 }.currentTime >= tStart & rawData{ 1 }.currentTime <= 0.94547 );	
idxS2 = find( rawData{ 2 }.currentTime >= tStart & rawData{ 2 }.currentTime <= 0.89115 );	

plot( rawData{1}.currentTime( idxS1 ), rawData{1}.geomXPositions( 4, idxS1 ), 'linewidth', 3, 'linestyle', '-', 'color',  color_arr( 1, : ) )
hold on
plot( rawData{2}.currentTime( idxS2 ), rawData{2}.geomXPositions( 4, idxS2 ), 'linewidth', 3, 'linestyle', '--', 'color',  color_arr( 1, : ) )

plot( rawData{1}.currentTime( idxS1 ), rawData{1}.geomYPositions( 4, idxS1 ), 'linewidth', 3, 'linestyle', '-', 'color',  color_arr( 2, : ) )
plot( rawData{2}.currentTime( idxS2 ), rawData{2}.geomYPositions( 4, idxS2 ), 'linewidth', 3, 'linestyle', '--', 'color',  color_arr( 2, : ) )
plot( rawData{1}.currentTime( idxS1 ), rawData{1}.geomZPositions( 4, idxS1 ), 'linewidth', 3, 'linestyle', '-', 'color',  color_arr( 3, : ) )
plot( rawData{2}.currentTime( idxS2 ), rawData{2}.geomZPositions( 4, idxS2 ), 'linewidth', 3, 'linestyle', '--', 'color',  color_arr( 3, : ) )

set( a, 'xlim', [0, 0.9352] )

%% -- (8C) Plot Comparison 3D

f = figure( ); a = axes( 'parent', f );
idx = 2;

tStarts = [0, 0];
tEnds   = [0.94547, 0.89115];

tStart = tStarts( idx );
tEnd   =   tEnds( idx );

idx_g = find( rawData{ idx }.output == min( rawData{ idx }.output )  );

hold on

tmpEL = pEL_func( 0.294, rawData{ idx }.qPos0( 2,idx_g ), rawData{ idx }.qPos0( 3,idx_g ) );
tmpEE = pEE_func( 0.294, 0.291, rawData{ idx }.qPos0( 2,idx_g ),  rawData{ idx }.qPos0( 3,idx_g ),  rawData{ idx }.qPos0( 4,idx_g ),  rawData{ idx }.qPos0( 5,idx_g) );


mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 500, ...       
                   'parent', a,   'LineWidth', 1,               ...     
                   'MarkerFaceColor',[0.4940 0.1840 0.5560], 'MarkerEdgeColor', [0.4940 0.1840 0.5560], ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );

for i = 1 : 3
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, idx_g ), ...
                 rawData{ idx }.geomYPositions( 2:4, idx_g ), ...
                 rawData{ idx }.geomZPositions( 2:4, idx_g ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ c.black, 1 ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, idx_g ), ...
                   rawData{ idx }.geomYPositions( 2:4, idx_g ), ...
                   rawData{ idx }.geomZPositions( 2:4, idx_g ), 800, ... 
                   'parent', a,   'LineWidth', 5, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

               
    p4 = scatter3( rawData{ idx }.geomXPositions( 5:end, idx_g ), ...
                   rawData{ idx }.geomYPositions( 5:end, idx_g ), ...
                   rawData{ idx }.geomZPositions( 5:end, idx_g ), 100, ... 
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0.75, 0, 0.75], ... c.purple_plum, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );

%     p4 = scatter3( [ 0, tmpEL_1( 1 ), tmpEE_1( 1 )], ...
%                    [ 0, tmpEL_1( 2 ), tmpEE_1( 2 )], ...
%                    [ 0, tmpEL_1( 3 ), tmpEE_1( 3 )], ...
%                    'parent', a,   'LineWidth', 3, ...
%                    'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0.75, 0, 0.75], ... c.purple_plum, ...
%                    'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );               
               
                      

    p6 = plot3(  [ 0, tmpEL( 1 ), tmpEE( 1 )], ...
                 [ 0, tmpEL( 2 ), tmpEE( 2 )], ...
                 [ 0, tmpEL( 3 ), tmpEE( 3 )], ...
                 'parent', a, ...
                'linewidth', 7, 'linestyle', '-', 'color', [ 0.5, 0.5, 0.5, 1 ] );               
    p5 = scatter3( [ 0, tmpEL( 1 ), tmpEE( 1 )], ...
                   [ 0, tmpEL( 2 ), tmpEE( 2 )], ...
                   [ 0, tmpEL( 3 ), tmpEE( 3 )], ...
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0.5, 0.5, 0.5], ... c.purple_plum, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );                    

               
end    
        
      
axis square

tmpLim = 2.7;               

     
set( a, 'xtick', [-2, 0, 2] ); set( a, 'xticklabel', ["-2", "\fontsize{50}X (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-2, 0, 2] ); set( a, 'yticklabel', ["-2", "\fontsize{50}Y (m)", "+2"] ); % ["-2", "Y[m]", "+2"] )
set( a, 'ztick', [-2, 0, 2] ); set( a, 'zticklabel', ["-2", "\fontsize{50}Z (m)", "+2"] ); % ["-2", "Z[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);

set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ], 'view', [0 ,0 ] )  %  [BACKUP] [Target #1] 16.3213    6.0865
xtickangle( 0 ); ytickangle( 0 )
if idx == 1
    exportgraphics( f, [ fig_dir,'S_fig5_wg.pdf'],'ContentType','vector' )
else
    exportgraphics( f, [ fig_dir,'S_fig5_wog.pdf'],'ContentType','vector' )
end
%% ==================================================================
%% (9-) Wrist Comparison
%% -- (9A) Data read

dir_name   = './myData/with_wrist_Target_1/optimization_result/';
file_names = { 'optimization_log_w_wrist.txt', 'optimization_log_wo_wrist.txt' };


N_data    = length( file_names ); 
data_list = cell( 1, N_data );    

for i = 1 : N_data
    file_name      = [dir_name, file_names{ i }];

    data_list{ i } = myTxtParse( file_name ); 
    
    % Printing out the idx, optimal value output and its input parameter
    opt_val  = min( data_list{ i }.output );
    idx      = find( data_list{ i }.output == opt_val );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
    
end

% Find and halt if the optimal val has no update 
opt_idx = zeros( 1, N_data );  % The index where the values stop.
tol  = 0.1;               % If the vector norm doesn't change that much, then halt the simulation
ntol = 15;

f = figure( ); a = axes( 'parent', f );
hold on

for i = 1 : N_data

    data     = data_list{ i };
    mov_norm = abs( diff( vecnorm( data.inputPars, 2 ) ) ); 
    
    tmp      = ( mov_norm <= tol ); % Boolean array which shows if mov_norm is within tol 
    cnt      = 0;
    for j = data.Iter  
        if tmp( j )
           cnt = cnt + 1; 
        else
           cnt = 0;
        end
        
        if cnt>= ntol
           opt_idx( i ) = j;
           break 
        end
        
    end

    plot( data.output( 1: opt_idx( i ) ), 'linewidth', 3 )
    disp( min( data.output( 1 : opt_idx( i ) ) ) )
end


% For saving the figure of the iteration
% exportgraphics( f, [ fig_dir,'S_fig2.eps'],'ContentType','vector')
% exportgraphics( f, [ fig_dir,'S_fig2.pdf'] )


% Displaying the best values within the opt_idx 
for i = 1 : N_data
    opt_val  = min( data_list{ i }.output( 1 : opt_idx( i ) ) );
    idx      = find( opt_val == data_list{ i }.output( 1 : opt_idx( i ) ), 1, 'first' );
    mov_pars = data_list{ i }.inputPars( :, idx )';
    fprintf( '[Target %d] [Optimal value] [%.5f] [idx] [%d]\n', i, opt_val, idx);
    fprintf( '[Target %d] [Optimal input pars] [%s] \n', i, join( string( mov_pars ), ', ' ) );
end

%% -- (9B) Reading the data 

dir_name   = './myData/with_wrist_Target_1/data_log/';
file_names = { 'data_log_wo_wrist.txt', 'data_log_w_wrist.txt' };


for i = 1 : N_data
    file_name      = [dir_name, file_names{ i }];
    
   rawData{ i } = myTxtParse( file_name  );
%     rawData{ i } = myTxtParse( ['data_log_dense_T', num2str( i ), '.txt']  );
   
   rawData{ i }.geomXPositions = rawData{ i }.geomXYZPositions( 1 : 3 : end , : );
   rawData{ i }.geomYPositions = rawData{ i }.geomXYZPositions( 2 : 3 : end , : );
   rawData{ i }.geomZPositions = rawData{ i }.geomXYZPositions( 3 : 3 : end , : );
end

%% -- (9C) Comparison with/without wrist
idx = 2;        % Choose type

idxS = find( rawData{ idx }.output == min( rawData{ idx }.output ), 1, 'first'  );

tIdx = [1, 42, idxS;
        1, 37, idxS ];

viewArr = [ 49.9456, 4.7355;
            49.9456, 4.7355];

alpha = [0.3, 0.5, 1.0];                                              % The alpha values of each screen shot   
f = figure( ); a = axes( 'parent', f, 'Projection','perspective' );
axis square; hold on;


color_arr = [0.4940, 0.1840, 0.5560; ...
             0.4940, 0.1840, 0.5560];
          
cTarget = color_arr( idx, : );

mTarget = scatter3( rawData{ idx }.geomXPositions( 1, 1 ), ...
                    rawData{ idx }.geomYPositions( 1, 1 ), ...
                    rawData{ idx }.geomZPositions( 1, 1 ), 500, ...        % Setting the handle of the ZFT Plot, 
                   'parent', a,   'LineWidth', 1,               ...       % For the main plot (s1)
                   'MarkerFaceColor', cTarget, 'MarkerEdgeColor', cTarget, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha',    1  );

      
for i = 1 : 3
    p1 = plot3(  rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                 rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), ...
                 'parent', a, ...
                'linewidth', 7, 'color', [ 0.2,0.2, 0.2, alpha( i ) ] );
            
    p2 = scatter3( rawData{ idx }.geomXPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 2:4, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 2:4, tIdx( idx, i ) ), 800, ... 
                   'parent', a,   'LineWidth',  5, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor', c.black, ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i )  );

               
%     p3 = plot3(  rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
%                  rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), ...
%                  'parent', a, ...
%                 'linewidth', 8, 'color', [ c.purple_plum, alpha( i ) ] );
            
    p4 = scatter3( rawData{ idx }.geomXPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomYPositions( 5:end, tIdx( idx, i ) ), ...
                   rawData{ idx }.geomZPositions( 5:end, tIdx( idx, i ) ), 100, ... 
                   'parent', a,   'LineWidth', 3, ...
                   'MarkerFaceColor', c.white, 'MarkerEdgeColor',  [0.75, 0, 0.75], ...[0.75, 0, 0.75], ...
                   'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', alpha(i)  );
               
               
end    


tmpLim = 2.4;               
set( a,   'XLim',   [ - tmpLim, tmpLim ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ - tmpLim, tmpLim ] , ...    
          'ZLim',   [ - tmpLim, tmpLim ] , ...
          'view',   viewArr( idx, : ) )  %  [BACKUP] [Target #1] 16.3213    6.0865
      

set( a, 'xtick', [-2, 0, 2] ); set( a, 'xticklabel', ["-2", "\fontsize{50}X (m)", "+2"] ); % ["-2", "X[m]", "+2"] )
set( a, 'ytick', [-2, 0, 2] ); set( a, 'yticklabel', ["-2", "\fontsize{50}Y (m)", "+2"] ); % ["-2", "Y[m]", "+2"] )
set( a, 'ztick', [-2, 0, 2] ); set( a, 'zticklabel', ["-2", "\fontsize{50}Z (m)", "+2"] ); % ["-2", "Z[m]", "+2"] )
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 )
exportgraphics( f,[fig_dir, 'SF7_',num2str(idx),'time_lapse.pdf'],'ContentType','vector')

%% ==================================================================
%% (10-) Simulation vs. Experiment
%% -- (10A) Elbow + Shoulder joint angle

raw_data_exp = load( './myData/Sim2Exp_Shared_NE/Sim2ExpData.mat' );

% A bit of trimming
fs = 667;
dt = 1/fs;

% Manually found initial/final posture 
ts = 1.65;
tf = 3.10;


N = length( raw_data_exp.StructOut.JAngles.Upper2Vertical' );

t_vec = 0:dt: (N-1)*dt;
idx = find( ts <= t_vec & tf >= t_vec );

t_vec_cal = t_vec( idx ) - min( t_vec( idx ) );

qpos_sim = [ raw_data_exp.StructOut.JAngles.Upper2Vertical( idx )'; raw_data_exp.StructOut.JAngles.Lower2Upper( idx )' ];
dist     = raw_data_exp.StructOut.Tip2TargetDistance( idx );

qpos_sim = qpos_sim * pi/180;

% f = figure( 1 ); a = axes( 'parent', f );
subplot( 2, 1, 1 )
plot( t_vec_cal, qpos_sim(1, :), 'linewidth', 5, 'color', 'k' )
hold on

color_sim = [0.0, 0.4470, 0.7410 ];
color_exp = [0.2, 0.2   , 0.2    ];

raw_data_sim = myTxtParse( './myData/Sim2Exp_Shared_NE/data_log_T4.txt' );
plot( raw_data_sim.currentTime, raw_data_sim.qPos( 1, :), 'linewidth', 5, 'color', color_sim, 'linestyle', '-' )
set( gca, 'xlim', [0, 1.3 ], 'ylim', [-2.2, 2.5], 'xtick', [0:0.2:1.2], 'xticklabel', [] )
% xlabel( 'Time (sec)' ); 
ylabel( 'Shoulder (rad)', 'fontsize', 40 ); 
[~, hobj, ~, ~]  = legend( 'Experiment' , 'Simulation', 'location', 'northwest', 'fontsize', 20 );
hl = findobj(hobj,'type','line');
set( hl, 'linewidth', 4 )



% f = figure( 2  ); a = axes( 'parent', f );
subplot( 2, 1, 2 )
plot( t_vec_cal, qpos_sim(2, :), 'linewidth', 5, 'color', 'k' )
hold on

raw_data_sim = myTxtParse( './myData/Sim2Exp_Shared_NE/data_log_T4.txt' );

plot( raw_data_sim.currentTime, raw_data_sim.qPos( 4, :), 'linewidth', 5, 'color', color_sim, 'linestyle', '-' )
set( gca, 'xlim', [0, 1.3 ], 'ylim', [0, 2.2]  )
% xlabel( 'Time (sec)' ); 
ylabel( 'Elbow (rad)', 'fontsize', 40 );  
[~, hobj, ~, ~]  = legend( 'Experiment' , 'Simulation', 'location', 'northwest' , 'fontsize', 20 );
hl = findobj(hobj,'type','line');
set( hl, 'linewidth', 4)

exportgraphics( gcf, [fig_dir, 'fig_9a_shoulder_elbow.pdf'],'ContentType','vector')

% exportgraphics( f,[fig_dir, 'fig_9a_elbow.pdf'],'ContentType','vector')

xlabel( 'Time (sec)', 'fontsize', 40 )
%% -- (10B) A Comparison of Simulation with Experiment

% Simulation Side
raw_data_sim = myTxtParse( './myData/Sim2Exp_Shared_NE/data_log_T4.txt' );

raw_data_sim.geomXPositions = raw_data_sim.geomXYZPositions( 1 : 3 : end , : );
raw_data_sim.geomYPositions = raw_data_sim.geomXYZPositions( 2 : 3 : end , : );
raw_data_sim.geomZPositions = raw_data_sim.geomXYZPositions( 3 : 3 : end , : );

t_start = 0;  t_end = 0.95;

% Find the start and end point of the data
idxS = find( raw_data_sim.currentTime >= t_start & raw_data_sim.currentTime <= t_end );	
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( 3 ); a = axes( 'parent', f, 'Projection','perspective'  );
axis equal; hold on;

% The end-effector index is the 4th one, 
% 1st one is target, 2nd one is shoulder, 3rd one is elbow and 4th the end-effector
x = raw_data_sim.geomXPositions( 4, idxS )';
y = raw_data_sim.geomYPositions( 4, idxS )';
z = raw_data_sim.geomZPositions( 4, idxS )';


p  = [ x, y, z ];                                                          % The position of the end-effector, N x 3
pC = mean( p );                                                            % The mean of the data
pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig( pn' * pn );

% The eigenvalues are ordered from low to high
% Getting the first eigenvectors and find the vectors that are orthogonal to it.

w = null( eigvecs( : , 1)' ); 
pC
eigvecs( :, 1 )


plot3( raw_data_sim.geomXPositions( 4, idxStart : idxEnd ), ...
       raw_data_sim.geomYPositions( 4, idxStart : idxEnd ), ...
       raw_data_sim.geomZPositions( 4, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 4, 'color', [ 0 0.4470 0.7410, 0.3] )

scatter3(  raw_data_sim.geomXPositions( 4, idxS ), ...
           raw_data_sim.geomYPositions( 4, idxS ), ...
           raw_data_sim.geomZPositions( 4, idxS ), 200, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0 0.4470 0.7410], ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           
       



tmp = 0;
for i = 1 : length( idxS )  % Brute force calculation of the distance.
%     ttmp = ( eigvecs(1,1) * ( x(i) - pC(1) ) ) + ( eigvecs(2,1) * ( y(i) - pC(2) ) )^2 + ( eigvecs(3,1) * ( z(i) - pC(3) )  )^2 ;
    ttmp = abs( ( eigvecs(1,1) * ( x( i ) - pC( 1 ) ) ) + ...
                ( eigvecs(2,1) * ( y( i ) - pC( 2 ) ) ) + ...
                ( eigvecs(3,1) * ( z( i ) - pC( 3 ) ) ) );
    tmp = tmp + ttmp^2;
end

sqrt( tmp/length( idxS ) )
                    
tmpLim = 0.55;      
[P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          

XX = pC( 1 ) + w( 1, 1 ) * P + w( 1, 2 ) * Q;                              
YY = pC( 2 ) + w( 2, 1 ) * P + w( 2, 2 ) * Q;                              
ZZ = pC( 3 ) + w( 3, 1 ) * P + w( 3, 2 ) * Q;


scatter3(  pC( 1 ) , pC( 2 ), pC( 3 ), 400, 'd',... 
           'parent', a,   'LineWidth', 6, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0 0.4470 0.7410], ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           

mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', [0 0.4470 0.7410], 'tipWidth', 0.03, 'stemWidth', 0.008 );

surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', [0 0.4470 0.7410], 'facealpha', 0.3 );


% Experiment Side 
% A bit of trimming
offset = 0.5;

% fs = 667;  dt = 1/fs;
fs = 667;  dt = 1/fs;
ts = 1.65; tf = 2.15;

N = length( raw_data_exp.StructOut.JAngles.Upper2Vertical' );

t_vec = 0 : dt : (N-1)*dt;
idx = find( ts <= t_vec & tf >= t_vec );

tmp = raw_data_exp.StructOut.CoordsInTargetRF.Hand';

tmp1 = [-1, 1];
set( a, 'xlim', tmp1, 'ylim', tmp1, 'zlim', tmp1, 'view', [49.9456, 4.7355] ); 
axis square

% Adding the Best-fit Plane
% Experiment
tmp = raw_data_exp.StructOut.CoordsInTargetRF.Hand';

color_exp = [0.2, 0.2, 0.2];

ts = 1.65;
tf = 2.15;

x_w_exp = 2 - tmp( 2, idx )';
y_w_exp = 0.2 + tmp( 1, idx )' + offset ;
z_w_exp = tmp( 3, idx )';

p  = [ x_w_exp, y_w_exp, z_w_exp ]; 
pC = mean( p );                                                            % The mean of the data
pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig( pn' * pn );

% The eigenvalues are ordered from low to high
% Getting the first eigenvectors and find the vectors that are orthogonal to it.

w = null( eigvecs( : , 1)' ); 
pC
eigvecs( :, 1 )

tmp = 0;
for i = 1 : length( idx )  % Brute force calculation of the distance.
%     ttmp = ( eigvecs(1,1) * ( x(i) - pC(1) ) ) + ( eigvecs(2,1) * ( y(i) - pC(2) ) )^2 + ( eigvecs(3,1) * ( z(i) - pC(3) )  )^2 ;
    ttmp= abs( ( eigvecs(1,1) * ( x_w_exp( i ) - pC( 1 ) ) ) + ...
                ( eigvecs(2,1) * ( y_w_exp( i ) - pC( 2 ) ) ) + ...
                ( eigvecs(3,1) * ( z_w_exp( i ) - pC( 3 ) ) ) );
    
    tmp = tmp + ttmp^2;
    tttmp( i ) = ttmp^2;
end


sqrt( tmp/length( idx ) )


tmpLim = 0.55;      
[P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          


XX = pC( 1 ) + w( 1, 1 ) * P + w( 1, 2 ) * Q;                              
YY = pC( 2 ) + w( 2, 1 ) * P + w( 2, 2 ) * Q;                              
ZZ = pC( 3 ) + w( 3, 1 ) * P + w( 3, 2 ) * Q;


hold on
scatter3(  pC( 1 ) , pC( 2 ), pC( 3 ), 400, 'd',... 
           'parent', a,   'LineWidth', 6, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_exp, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );      
       
ttmp = 2 * round( 1/60/(1/fs ) )
       
scatter3(  x_w_exp( 1 : ttmp : end ), y_w_exp( 1 : ttmp : end ), z_w_exp( 1 : ttmp : end ),200, ...
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_exp, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );       
       
surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', color_exp, 'facealpha', 0.3 );

mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', color_exp, 'tipWidth', 0.03, 'stemWidth', 0.008 );
tmpLim2 = 0.8;

set( a,   'XLim',   [ -tmpLim2, tmpLim2 ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ -tmpLim2 + offset/2, tmpLim2 + offset/2] , ...    
          'ZLim',   [ -tmpLim2, tmpLim2 ] , ...
          'view',   [ 118    4.3877] )  
set( a, 'xtick', [-0.5, 0, 0.5] ); set( a, 'xticklabel', ["", "\fontsize{43}X (m)", ""] )
set( a, 'ytick', [ offset/2 ] ); set( a, 'yticklabel', ["\fontsize{43}Y (m)"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["\fontsize{35}-0.5", "\fontsize{43}Z (m)", "\fontsize{35}+0.5"] ); % ["-2", "X[m]", "+2"] )        
set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )      
      
axis square      
exportgraphics( f,[fig_dir, 'fig_9a_planarity.pdf'],'ContentType','vector')
% exportgraphics( f,[fig_dir, 'SF10_BFP.pdf'],'ContentType','vector')

%% -- (10B-2) A Comparison of Simulation with Experiment

% Simulation Side
raw_data_sim = myTxtParse( './myData/Sim2Exp_Shared_NE/data_log_T4.txt' );

raw_data_sim.geomXPositions = raw_data_sim.geomXYZPositions( 1 : 3 : end , : );
raw_data_sim.geomYPositions = raw_data_sim.geomXYZPositions( 2 : 3 : end , : );
raw_data_sim.geomZPositions = raw_data_sim.geomXYZPositions( 3 : 3 : end , : );

t_start = 0;  t_end = 0.95;

% Find the start and end point of the data
idxS = find( raw_data_sim.currentTime >= t_start & raw_data_sim.currentTime <= t_end );	
idxStart = min( idxS ); idxEnd = max( idxS );

f = figure( 3 ); a = axes( 'parent', f  );
axis equal; hold on;

% The end-effector index is the 4th one, 
% 1st one is target, 2nd one is shoulder, 3rd one is elbow and 4th the end-effector
x = raw_data_sim.geomXPositions( 4, idxS )';
y = raw_data_sim.geomYPositions( 4, idxS )';
z = raw_data_sim.geomZPositions( 4, idxS )';


p  = [ x, y, z ];                                                          % The position of the end-effector, N x 3
pC = mean( p );                                                            % The mean of the data
pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig( pn' * pn );

% The eigenvalues are ordered from low to high
% Getting the first eigenvectors and find the vectors that are orthogonal to it.

w = null( eigvecs( : , 1)' ); 
pC
eigvecs( :, 1 )


plot3( raw_data_sim.geomXPositions( 4, idxStart : idxEnd ), ...
       raw_data_sim.geomYPositions( 4, idxStart : idxEnd ), ...
       raw_data_sim.geomZPositions( 4, idxStart : idxEnd ), ... 
      'parent', a,   'LineWidth', 4, 'color', [ 0 0.4470 0.7410, 0.3] )

scatter3(  raw_data_sim.geomXPositions( 4, idxS ), ...
           raw_data_sim.geomYPositions( 4, idxS ), ...
           raw_data_sim.geomZPositions( 4, idxS ), 200, ... 
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0 0.4470 0.7410], ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           
       



tmp = 0;
for i = 1 : length( idxS )  % Brute force calculation of the distance.
%     ttmp = ( eigvecs(1,1) * ( x(i) - pC(1) ) ) + ( eigvecs(2,1) * ( y(i) - pC(2) ) )^2 + ( eigvecs(3,1) * ( z(i) - pC(3) )  )^2 ;
    ttmp = abs( ( eigvecs(1,1) * ( x( i ) - pC( 1 ) ) ) + ...
                ( eigvecs(2,1) * ( y( i ) - pC( 2 ) ) ) + ...
                ( eigvecs(3,1) * ( z( i ) - pC( 3 ) ) ) );
    tmp = tmp + ttmp^2;
end

sqrt( tmp/length( idxS ) )
                    
tmpLim = 0.55;      
[P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          

XX = pC( 1 ) + w( 1, 1 ) * P + w( 1, 2 ) * Q;                              
YY = pC( 2 ) + w( 2, 1 ) * P + w( 2, 2 ) * Q;                              
ZZ = pC( 3 ) + w( 3, 1 ) * P + w( 3, 2 ) * Q;


scatter3(  pC( 1 ) , pC( 2 ), pC( 3 ), 400, 'd',... 
           'parent', a,   'LineWidth', 6, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', [0 0.4470 0.7410], ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );           

mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', [0 0.4470 0.7410], 'tipWidth', 0.03, 'stemWidth', 0.008 );

surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', [0 0.4470 0.7410], 'facealpha', 0.3 );


% Experiment Side 
% A bit of trimming
offset = 0.5;

fs = 667;  dt = 1/fs;
ts = 1.65; tf = 2.15;

N = length( raw_data_exp.StructOut.JAngles.Upper2Vertical' );

t_vec = 0 : dt : (N-1)*dt;
idx = find( ts <= t_vec & tf >= t_vec );

tmp = raw_data_exp.StructOut.CoordsInTargetRF.Hand';

color_exp = [0.2, 0.2, 0.2];

tmp1 = [-1, 1];
set( a, 'xlim', tmp1, 'ylim', tmp1, 'zlim', tmp1, 'view', [49.9456, 4.7355] ); 
axis square

% Adding the Best-fit Plane
% Experiment
tmp = raw_data_exp.StructOut.CoordsInTargetRF.Hand';

ts = 1.65;
tf = 2.15;

x_w_exp = 2 - tmp( 2, idx )';
y_w_exp = 0.2 + tmp( 1, idx )' + offset ;
z_w_exp = tmp( 3, idx )';

p  = [ x_w_exp, y_w_exp, z_w_exp ]; 
pC = mean( p );                                                            % The mean of the data
pn = p - pC;                                                               % Centralized data
[eigvecs, eigvals] = eig( pn' * pn );

% The eigenvalues are ordered from low to high
% Getting the first eigenvectors and find the vectors that are orthogonal to it.

w = null( eigvecs( : , 1)' ); 
pC
eigvecs( :, 1 )

tmp = 0;
for i = 1 : length( idx )  % Brute force calculation of the distance.
%     ttmp = ( eigvecs(1,1) * ( x(i) - pC(1) ) ) + ( eigvecs(2,1) * ( y(i) - pC(2) ) )^2 + ( eigvecs(3,1) * ( z(i) - pC(3) )  )^2 ;
    ttmp = abs( ( eigvecs(1,1) * ( x_w_exp( i ) - pC( 1 ) ) ) + ...
                ( eigvecs(2,1) * ( y_w_exp( i ) - pC( 2 ) ) ) + ...
                ( eigvecs(3,1) * ( z_w_exp( i ) - pC( 3 ) ) ) );
    tmp = tmp + ttmp^2;
end

sqrt( tmp/length( idx ) )


tmpLim = 0.55;      
[P,Q] = meshgrid( -tmpLim + 0.2: 0.1 : tmpLim + 0.1);          


XX = pC( 1 ) + w( 1, 1 ) * P + w( 1, 2 ) * Q;                              
YY = pC( 2 ) + w( 2, 1 ) * P + w( 2, 2 ) * Q;                              
ZZ = pC( 3 ) + w( 3, 1 ) * P + w( 3, 2 ) * Q;


hold on
scatter3(  pC( 1 ) , pC( 2 ), pC( 3 ), 400, 'd',... 
           'parent', a,   'LineWidth', 6, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_exp, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );      
       
ttmp = 2 * round( 1/60/(1/fs ) )
       
scatter3(  x_w_exp( 1 : ttmp : end ), y_w_exp( 1 : ttmp : end ), z_w_exp( 1 : ttmp : end ),200, ...
           'parent', a,   'LineWidth', 4, ...
           'MarkerFaceColor', c.white, 'MarkerEdgeColor', color_exp, ...
           'MarkerFaceAlpha', 1      , 'MarkerEdgeAlpha', 1  );       
       
surf( XX, YY, ZZ, 'parent', a, 'edgecolor', 'none', 'facecolor', color_exp, 'facealpha', 0.3 );

mArrow3( pC, pC - 0.3 * eigvecs( : , 1 )', 'color', color_exp, 'tipWidth', 0.03, 'stemWidth', 0.008 );
tmpLim2 = 0.8;

set( a,   'XLim',   [ -tmpLim2, tmpLim2 ] , ...                             % Setting the axis ratio of x-y-z all equal.
          'YLim',   [ -tmpLim2 + offset/2, tmpLim2 + offset/2] , ...    
          'ZLim',   [ -tmpLim2, tmpLim2 ] , ...
          'view',   [ 91.8260, 0] )  
set( a, 'xtick', [] ); set( a, 'xticklabel', ["", "\fontsize{43}X (m)", ""] )
set( a, 'ytick', [ offset/2 ] ); set( a, 'yticklabel', ["\fontsize{43}Y (m)"] )
set( a, 'ztick', [-0.5, 0, 0.5] ); set( a, 'zticklabel', ["", "", ""] )



set(a,'LineWidth',3.0 ); set(a, 'TickLength',[0.01, 0.03]);
xtickangle( 0 ); ytickangle( 0 ); ztickangle( 0 )      
      
axis square      

exportgraphics( f,[fig_dir, 'fig_9ab_planarity.pdf'],'ContentType','vector')
% exportgraphics( f,[fig_dir, 'SF10_BFP.pdf'],'ContentType','vector')

%% 
tmpH = raw_data_exp.StructOut.CoordsInTargetRF.Hand';
tmpE = raw_data_exp.StructOut.CoordsInTargetRF.Elbow';
tmpS = raw_data_exp.StructOut.CoordsInTargetRF.Shoulder';
tmpW = raw_data_exp.StructOut.CoordsInTargetRF.w1'; 
hold on
plot3( tmpH( 1, : ), tmpH( 2, : ), tmpH( 3, : ) )
plot3( tmpE( 1, : ), tmpE( 2, : ), tmpE( 3, : ) )
plot3( tmpS( 1, : ), tmpS( 2, : ), tmpS( 3, : ) )
% plot3( tmpW( 1, : ), tmpW( 2, : ), tmpW( 3, : ) )
axis square


%% -- (11A) Distribution of the Optimal Parameters - Reading Data

% Using 600 iterations
N = 600;
for i = 1 : 3
    data_raw{ i } = myTxtParse( ['./myData/optimization_process_3_new/optimization_log_T',num2str( i + 3 ), '_longer.txt' ] );
    data_raw{ i }.opt_idx = find( data_raw{ i }.output( 1: N ) == 0.1 );
end

dir_name  = './myData/optimization_process_3_new/';
for i = 4 : 6
    file_name      = [ dir_name, 'optimization_log_T', num2str( i - 3 ), '.txt' ];
    data_raw{ i } = myTxtParse( file_name );
end


%% -- (11B) Parsing the data and retriving the best value

% Find and halt if the optimal val has no update 
opt_idx = zeros( 1, 6 );  % The index where the values stop.
tol  = 0.1;               % If the vector norm doesn't change that much, then halt the simulation
ntol = 15;

f = figure( ); a = axes( 'parent', f );
hold on

for i = 1 : 6   % For target 1, 2 and 3

    data     = data_raw{ i };
    mov_norm = abs( diff( vecnorm( data.inputPars, 2 ) ) ); 
    
    tmp      = ( mov_norm <= tol ); % Boolean array which shows if mov_norm is within tol 
    cnt      = 0;
    for j = data.Iter  
        if tmp( j )
           cnt = cnt + 1; 
        else
           cnt = 0;
        end
        
        if cnt>= ntol
           opt_idx( i ) = j;
           break 
        end
        
    end

%     plot( data.output( 1: opt_idx( i ) ), 'linewidth', 3 )
%     disp( min( data.output( 1 : opt_idx( i ) ) ) )
end

% For targets that is within reach
subplot( 2, 1, 1 )
hold on
plot( data_raw{ 1 }.Iter( 1 : opt_idx( 1 ) ), data_raw{ 1 }.output( 1 : opt_idx( 1 ) ) - 0.1, 'linewidth', 3, 'color', [0.0000, 0.4470, 0.7410] )
plot( data_raw{ 2 }.Iter( 1 : opt_idx( 2 ) ), data_raw{ 2 }.output( 1 : opt_idx( 2 ) ) - 0.1, 'linewidth', 3, 'color', [0.8500, 0.3250, 0.0980] )
plot( data_raw{ 3 }.Iter( 1 : opt_idx( 3 ) ), data_raw{ 3 }.output( 1 : opt_idx( 3 ) ) - 0.1, 'linewidth', 3, 'color', [0.9290, 0.6940, 0.1250] )
set( gca, 'ylim', [-0.1, 3.5], 'xlim', [0, 160],  'fontsize', 30 )
[~, hobj, ~, ~] = legend( 'Target 1', 'Target 2', 'Target 3', 'fontsize', 30 );
ht = findobj(hobj,'type','line');
set(ht,'Linewidth',12);

subplot( 2, 1, 2 )
hold on
plot( data_raw{ 4 }.Iter( 1 : opt_idx( 4 ) ), data_raw{ 4 }.output( 1 : opt_idx( 4 ) ) , 'linewidth', 3, 'color', [0.4940 0.1840 0.5560] )
plot( data_raw{ 5 }.Iter( 1 : opt_idx( 5 ) ), data_raw{ 5 }.output( 1 : opt_idx( 5 ) ) , 'linewidth', 3, 'color', [0.4660 0.6740 0.1880] )
plot( data_raw{ 6 }.Iter( 1 : opt_idx( 6 ) ), data_raw{ 6 }.output( 1 : opt_idx( 6 ) ) , 'linewidth', 3, 'color', [0.6350 0.0780 0.1840] )
[~, hobj, ~, ~] = legend( 'Target 4', 'Target 5', 'Target 6', 'fontsize', 30 );
  
set( gca, 'ylim', [-0.1, 3.5], 'xlim', [0, 260],  'fontsize', 30 )
ht = findobj(hobj,'type','line');
set(ht,'Linewidth',12);
han=axes(gcf,'visible','off'); 
han.Title.Visible='on';
han.XLabel.Visible='on';
han.YLabel.Visible='on';

% Add Text
xlabel( han, 'Iteration (-)' );
ylabel( han, '{\it{L^*}}(m)' );

% For saving the figure of the iteration
exportgraphics( f, [ fig_dir,'fig2.eps'],'ContentType','vector')
exportgraphics( f, [ fig_dir,'fig2.pdf'] )

%% -- (11C) Plotting the data distribution
% Plotting the distribution 
tmp = { '\phi_{i,1}', '\phi_{i,2}', '\phi_{i,3}', '\phi_{i,4}', ...
        '\phi_{f,1}', '\phi_{f,2}', '\phi_{f,3}', '\phi_{f,4}', 'D', } ;
    
    
color_arr = [      0, 0.4470, 0.7410; ...
              0.8500, 0.3250, 0.0980; ...
              0.9290, 0.6940, 0.1250  ];

    
opt_mov_pars = cell( 1, 3 );    

ub = [ -0.5 * pi, -0.5 * pi, -0.5 * pi,        0, 0.1 * pi, -0.5 * pi, -0.5 * pi,        0, 0.4 ];
lb = [ -0.1 * pi,  0.5 * pi,  0.5 * pi, 0.9 * pi, 1.0 * pi,  0.5 * pi,  0.5 * pi, 0.9 * pi, 1.5 ] ;

for i = 1 : 3
    
    data_raw{ i }.opt_idx = find( data_raw{ i }.output( 1: opt_idx( i ) ) == 0.1 );
    idx = data_raw{ i }.opt_idx;
    opt_mov_pars{ i } = data_raw{ i }.inputPars( :, idx );
    opt_mov_pars{ i } = uniquetol( opt_mov_pars{ i }', 0.1, 'ByRows', true )';
%     f = figure( i ); a = axes( 'parent', f );
    
    subplot( 3, 1, i )
    hold on
    
    for j = 1 : 9
       plot( j, opt_mov_pars{ i }( j, : ) , 'o', 'linewidth', 4, 'markeredgecolor', color_arr( i, : ), 'markersize', 6, 'markerfacecolor', color_arr( i, : ) )
       plot( j, ub( j ) , '_', 'color', 'k', 'linewidth', 2 )
       plot( j, lb( j ) , '_', 'color', 'k', 'linewidth', 2 )
    end
    
    
    
    
    hold off
    set( gca, 'ylim', [-2.0, 3.4], 'xticklabel', [], 'xtick', [1:9], 'xlim', [0, 10] )
%     title( [ 'Target ', num2str( i  ) ] )
%     xlabel( 'Optimal Movement Parameters' )
    disp( mean( opt_mov_pars{i}' ) )
%     exportgraphics( f, ['./figure', num2str( i ),'.pdf' ],'ContentType','vector')
    legend( ['Target ', num2str( i ) ], 'location', 'northwest' )
end
set( gca, 'xtick', [1:9], 'xticklabel', tmp )

han=axes(gcf,'visible','off'); 
han.Title.Visible='on';
han.XLabel.Visible='on';
han.YLabel.Visible='on';

ylabel( 'Values (rad, s)' )

exportgraphics( gcf, './myFigures/value_distb.pdf','ContentType','vector')
exportgraphics( gcf, './myFigures/value_distb.eps','ContentType','vector')


A = [0, 1; 0, 0];
B = [0;    1];
C = [1,    0];


% Subproblem 2 - want to place the poles to -1 \pm j.
K = place( A,  B,  [ -1+1i, -1-1i ] );            % result is K = [2,2];

% Subproblem 3 - the problem  becomes how to place A-LC of the observer dynamics
% Placing poles of A - LC is same with A^T - C^TL^T

L = place( A', C', [ -3+3i, -3-3i ] );            % result is L = [6,18]';
L = L';                                           % Just matching the shape

% (Case 2) Using Observer Dynamics.
A2 = [ A - B *K, B*K; zeros( 2, 2 ), A-L*C ]; 
B2 = [ B; zeros( 2,1 ) ];
C2 = [0, 1, 0, 0];

[num, den] = ss2tf( A2, B2, C2, 0 )
[P, Z] = pzmap( tf( num, den) );

plot( real( P ), imag( P ), 'x', 'markersize', 20, 'linewidth', 6 )
hold on
plot( real( Z ), imag( Z ), 'o', 'markersize', 20, 'linewidth', 6 )
xline( 0, 'linewidth', 3 );
yline( 0, 'linewidth', 3 );
xlabel( 'Real' ); ylabel( 'Imag' )
set( gca, 'xlim', [-4, 1], 'ylim', [-4, 4] )

mySaveFig( gcf, 'fig0' )
%%
% sys2 = ss( A2, B2, C2, 0 ); 
% [y,t,x] = initial( sys2, [ 0, 1, 1, 0.1] )
% 
% f = figure( ); a = axes( 'parent', f );
% p2 = plot( t, x(:,2)+x(:,4), '-', 'parent', a );
% hold on
% p2 = plot( t, x(:,2), '-', 'parent', a );
% title( 'Angular velocity \omega' )
% xlabel( 'Time (sec)' )
% ylabel( '$\omega$, $$\hat{\omega}$$ (rad/s)', 'interpreter', 'latex' )
% legend( {'$$\omega$$', '$$\hat{\omega}$$'}, 'Interpreter', 'Latex' )
% 
%  mySaveFig( f, 'fig1' )
% 
% f = figure( ); a = axes( 'parent', f );
% p2 = plot( t, x(:,1)+x(:,3), '-', 'parent', a );
% hold on
% p2 = plot( t, x(:,1), '-', 'parent', a );
% title( 'Angular Displacement \theta' )
% xlabel( 'Time (sec)' )
% ylabel( '$\theta$, $$\hat{\theta}$$ (rad)', 'interpreter', 'latex' )
% legend( {'$$\theta$$', '$$\hat{\theta}$$'}, 'Interpreter', 'Latex' )
% 
% mySaveFig( f, 'fig2' )
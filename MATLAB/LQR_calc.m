% [REF1] https://www.ashwinnarayan.com/post/cartpole-dynamics/ <-- The equation's signs are WRONG!!!!!
% [REF2] https://www.mathworks.com/help/control/ref/lqr.html
% [REF3] https://metr4202.uqcloud.net/tpl/t8-Week13-pendulum.pdf
mc = 0.03;
mp = 0.03;
l  = 0.60; 
g  = 9.81;

A = [ 0,                  0, 1, 0;
      0,                  0, 0, 1;
      0,         -mp/mc * g, 0, 0;
      0, (mp + mc)/mc/l * g, 0, 0];
  
B = [0; 0; 1/mc; -1/mc/l];
Q = diag( [1, 1, 1, 1] ); % Penalize State
R =         0.01; % Penalize input
N = zeros( 4, 1 );

[K,~,~] = lqr(A,B,Q,R,N);



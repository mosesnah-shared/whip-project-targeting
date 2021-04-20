function sym_out = myFunctionize( mySym, sym_from, sym_to )
% Generate the ellipsoid mesh array, a "level-curve method", x^T (arrs) x <= 1
%   input:  [1] mySym    (sym)    The symbolic expression that we are going to change
%           [2] sym_from (string) From, the expressions that will be changed.
%           [3] sym_to   (string) To,   how the expressions are changed. 
%
%   output: [1] sym_out  (func) output is function handle.

    tmp = arrayfun( @char,   mySym, 'uniform', 0 );
    tmp = replace( tmp, sym_from, sym_to );

    sym_out = matlabFunction( str2sym( tmp ) );
    
end


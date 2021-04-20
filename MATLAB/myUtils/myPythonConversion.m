function myPythonConversion( mySim, simName, oldS, newS )
% Generate the ellipsoid mesh array, a "level-curve method", x^T (arrs) x <= 1
%   input:  [1] mySym       (sym) The symbolic expression that we are going to change
%           [2] simName  (string) sim Name
%           [3] sym_from (string) From, the expressions that will be changed.
%           [4] sym_to   (string) To,   how the expressions are changed. 
%
%   output: Just spitting out the string on the command window so to Ctrl C + V onto python 

    tmp = arrayfun( @char, mySim, 'uniform', 0 );

    
    for i = 1 : length( oldS )
        tmp = strrep( tmp, oldS{ i }, newS{ i } );                           % Replacing 
    end

    [nr, nc] = size( mySim );

    for i = 1 : nr
        for j = 1 : nc 
            fprintf( 'self.%s[%d, %d] = %s\n', simName, i-1, j-1, tmp{ i, j }  )
        end
    end
end


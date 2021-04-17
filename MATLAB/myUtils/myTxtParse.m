function rawData = myTxtParse( fileName )
% myTxtParse for parsing the txt file named "fileName"
%
% =============================================================== %
% [CREATED BY]: Moses C. Nah
% [   DATE   ]: 14-June-2020
% =============================================================== %
%
% =============================================================== %
% [DESCRIPTION]
%    - Function for parsing the txt file.
% =============================================================== %
%
% =============================================================== %
% [INPUT] 
%   (1) fileName (string)
%       -  The txt file name aimed to be parsed
%          The syntax of the txt file should be as following:
%          [DATANAME] ACTUAL DATA
%          The first string within the square bracket ([]), is the data name, followed by the actual numerical data. 
%          This function will save a structure called rawData, where the fields are DATENAME
%
% =============================================================== %
% [OUTPUT] 
%   (1) rawData (structure)
%       -  The data name and its actual data. 
%
% =============================================================== %
%
% [REMARKS]  ADD DETAILS
%
% =============================================================== %
%
% =============================================================== %
% SEE ALSO testHelpFunction 
%
% =============================================================== %

fid     = fopen( fileName );                                               % Opening the txt file with the name "txtName"
rawData = struct();

while( ~feof( fid ) )

    tline = fgetl( fid );                                                  % Get the txt file

    tmp = regexp( tline , '\[(.*?)\]', 'match' );                          % [REF] https://stackoverflow.com/questions/2403122/regular-expression-to-extract-text-between-square-brackets
                                                                           % Extracting out the string or float inside a square bracket.
                                                                           % \[    : [ is a meta char and needs to be escaped if you want to match it literally.
                                                                           % (.*?) : match everything in a non-greedy way and capture it.
                                                                           % \]    : ] is a meta char and needs to be escaped if you want to match it literally. 

    tmpField = tmp{ 1 }( 2 : end - 1 );                                    % Getting the name of the data
    tmpField = tmpField( ~isspace( tmpField ) );                           % Discarding the blank space within the string
    tmpValue = cellfun( @str2double, regexp( tline , '[+-]?\d+\.?\d*', 'match' ) );    
                                                                           % Getting the value array of the data
                                                                           
    if isempty( tmpValue )  % If value isn't a value but a string (e.g., inf, nan)
        tmpValue = NaN;
    end
    
    if ~isfield( rawData, tmpField )                                       % If new field, add to rawData list
         rawData.( tmpField ) = tmpValue';
    else    
         rawData.( tmpField ) = horzcat( rawData.( tmpField ), tmpValue' );% Append the data
    end

end

end
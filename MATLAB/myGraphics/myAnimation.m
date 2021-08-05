classdef myAnimation < handle
%
%  my3DAnimation class for setting up the 3D animation with the input data.
% 
% % =============================================================== %
%   [DESCRIPTION]
%
%
% % =============================================================== %
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     08-June-2020    
% % =============================================================== %


% % =============================================================== %
%   [REFERENCES]
%   myAnimation class for setting up the 3D animation with the input data.
% 
% % =============================================================== %

% % =============================================================== %
% [START] 

    properties ( SetAccess = private )
        
        % ================================================================= % 
        % [Figure and Axes Configuration]
        % suffix m  stands for main
        % suffix s1 stands for side plot #1 (Lower Right)
        % suffix s2 stands for side plot #2 (Upper Right)
        
                                           % [Configuration #1] A single big plot
        pos1m  = [0.08 0.10 0.84 0.80];    %   Position/size for the main plot before adding plot 
        
                                           % [Configuration #2] A big plot on the left, two plots on the upper/lower right.
        pos2   = [0.08 0.08 0.42 0.82; ... %   Position/size for the main plot - 3D real time plot
                  0.58 0.08 0.40 0.37; ... %   Position/size for the under-sub-plot, drawn in the lower section
                  0.58 0.55 0.40 0.40]     %   Position/size for the above-sub-plot, drawn in the upper section.

    end
    
    properties ( SetAccess = public )
        % ================================================================ %
        % [Syntax]
        % (1) Graphic Handle
        %     - h Stands for handle
        %     - F, A, L, M, E Stands for Figure, Axes, Line, Markers, Ellipse Handle, respectively.
        %     - m, s1, s2 stands for main, side1, side2, respectively.
        %     - s1Z, Z suffix stands for "Zoomed" Plot, which is special 
        % (2) Position Information 
        %     - p prefix stands for XYZ position information.
        %     - If pX, then position of X.
        % (3) Indexing Information 
        %     - The indexing information is important for "connect" method 
        %     - "idx" prefix stands for indexing
        % ================================================================ %
        
        % ================================================================ %
        % [START OF DEFINITION]
        
        tVec; tStep;            % Time vector and step of the simulation.
        simStep = 1;            % The step number of the simulation.
        
        vidRate = 0;            % Default as zero
        
                                % [The handle of the Figure and Title]
        hFigure = []            % The handle of the whole figure
        hTitle  = []            % The  title of the whole figure.

                                % [The handle of the axes]
        % The list of axes, 1,2,3 is ordered as main, side1, side2
        hAxes        = {        gobjects(0),       gobjects(0),       gobjects(0) };
        hGraphicObjs = {        gobjects(0),       gobjects(0),       gobjects(0) };
        graphicObjs  = {   myGraphics.empty,  myGraphics.empty,  myGraphics.empty };    

        isZoomed   = [           false,          false,               false]; % Check whether the plot is a zoomed-plot or not
        zoomIdx; zoomSize;
    end
    
    methods
                
        function obj = myAnimation( tStep, gObjs, varargin )
            %[CONSTRUCTOR #1] Construct an instance of this class
            %   (1) tVec [sec]
            %       -  The time vector of the simulation
            %   (2) gObjs [graphic objects cell array] 
            %       -  The class array of "myMarker" class. 
  
                        
            % If the size of the data are all the same.
            % Setting the default time step and its corresponding time vector.
            obj.tStep = tStep;
            obj.tVec  = tStep * (0 : length( gObjs( 1 ).XData ) - 1 );     % Taking out 1 to match the size of data        
            
            % Setting the default figure and Axes for the plot
            obj.hFigure     = figure();
            obj.hAxes{ 1 }  = subplot( 'Position', obj.pos1m, 'parent', obj.hFigure );    
            obj.hTitle      = title( obj.hAxes{ 1 }, sprintf( '[Time] %5.3fs', obj.tVec( 1 ) ), 'fontsize',30 );
            hold( obj.hAxes{ 1 },'on' ); axis( obj.hAxes{ 1 } , 'equal' )             

            for g = gObjs
                obj.addGraphicObject( 1, g )                              
            end
                        
        end
        
        function adjustFigures( obj, idx )
             %adjustFigures: adjusting the positioning of figures for multiple plots
             % This function is more of a internal function.
             % [INPUT]
             %    (1) idx, 1 (Main), 2 (Lower-right), 3 (Upper-right)
             
            if  ( idx == 2 ) && ( isempty( obj.hAxes{ 2 } ) )
                set( obj.hAxes{ 1 }, 'Position', obj.pos2( 1,: ) ); 
                
                obj.hAxes{ 2 } = subplot( 'Position', obj.pos2( 2, : ), 'parent', obj.hFigure );
                hold( obj.hAxes{ 2 },    'on' );  %axis( obj.hAxes{ 2 }, 'equal' ); 
            end   
            
            if  ( idx == 3 ) && ( isempty( obj.hAxes{ 3 } ) )
                set( obj.hAxes{ 1 }, 'Position', obj.pos2( 1,: ) ); 
                
                obj.hAxes{ 3 } = subplot( 'Position', obj.pos2( 3, : ), 'parent', obj.hFigure );
                hold( obj.hAxes{ 3 },    'on' );  axis( obj.hAxes{ 3 }, 'equal' ); 
            end   
            
            
        end
        
        function addGraphicObject( obj, idx, myGraphics, varargin )
             %addGraphicObject: adding a single graphical object to the plot
             % [INPUT]
             %   (1) idx [integer]
             %       -  1 (Main), 2 (Lower-right), 3 (Upper-right)
             %   (2) myGraphics [marker array] 
             %       -  The graphics class that we are aimed to add
             %       -  Type of Graphic Objects
             %          (a) myMarkers
             %          (b) myLines
             %          (c) myArrow             
             %    (3) varagin
             %     - [TO BE ADDED]             

            obj.adjustFigures( idx )                                       % Change the configuration of the plots if side plots are not drawn
            h2Draw = obj.hAxes{ idx };                                     % Getting the axes to do the plot
            
            for g = myGraphics
                h = g.create( h2Draw );                                    % Passing the axis that the object should be drawn.
                
                obj.hGraphicObjs{ idx }( end + 1 ) = h;                    % Appending the handle of the graphic objects 
                 obj.graphicObjs{ idx }( end + 1 ) = g;                    % Appending the graphic objects                 
            end
            
        end
        
        function connectMarkers( obj, idx, whichMarkers, varargin )
            %connect: connecting markers with lines in the main plot
            % =============================================================== %
            % [INPUTS]
            %   (1) idx [integer]
            %       -  The plot that are aimed to 
            %   (1) whichMarkers (string) List
            %       - the index or the name of the markers that are aimed to be connected
            %   (2) varargin
            %       - [TO BE ADDED]              
            
            % Temporary Saving the list of names of the markers for the connect
            obj.adjustFigures( idx )            % Change the configuration of the plots if side plots are not drawn
            
            N = length( whichMarkers );
            
            % Allocating the x,y and z markers
            tmpx = zeros( N, length( obj.tVec) ); 
            tmpy = tmpx; 
            tmpz = tmpx;
                            
            tmp = [ obj.graphicObjs{ idx }.name ];

            for i = 1 : N
                tmpi = find( strcmp( tmp, whichMarkers( i ) ) );       % Getting the index of the marker which matches the string name
                    
                tmpx( i,: ) = obj.graphicObjs{ idx }( tmpi ).XData;
                tmpy( i,: ) = obj.graphicObjs{ idx }( tmpi ).YData;
                tmpz( i,: ) = obj.graphicObjs{ idx }( tmpi ).ZData;                    
                    
            end

            line = myLine( 'XData', tmpx, 'YData', tmpy, 'ZData', tmpz, varargin{ : } );
            obj.addGraphicObject( idx, line );
                                         
        end        

        function run( obj, vidRate, duration, isVidRecord, videoName )
            %run: running the a whole animation and saving the video if defined.
            % [INPUTS]
            %   (1) vidRate 
            %       - video Rate of the animation.
            %       - if vidRate is 0.5, then the video is two times slower. 
            %       - if vidRate is 1.0, then the video is at real time speed.
            %       - if vidRate is 2.0, then the video is two times faster. 
            %   (2) duration
            %       - Duration of the animation        
            %
            %   (3) isVidRecord ( boolean )
            %       - true or false, if false, simply showing the animation.
            %
            %   (4) videoName (string)
            %       - name of the video for the recording.

            obj.vidRate = vidRate;
            
            if duration >= max( obj.tVec )
               duration = max( obj.tVec ); 
            end
                
            if vidRate >=1 
                fps = 30;
            else
                fps = 30 * vidRate; 
            end

            
            if isVidRecord                                                 % If video record is ON
                
                writerObj = VideoWriter( videoName, 'MPEG-4' );            % Saving the video as "videoName" 
                writerObj.FrameRate = fps;                                 % How many frames per second.
                open( writerObj );                                         % Opening the video write file.

            end    
            
            N    = round( duration/ obj.tStep );
            step = round( vidRate * ( 1 / obj.tStep / fps ) );              % Setting 60 fps - 1 second as default!


            if step == 0                                                   % In case the step is too small, then set the simStep as 1
               step = 1; 
            end            
            

            for i = 1 : step : N
                
                obj.goto( i )                                              % Run a single step of the simulation
                if isVidRecord                                             % If videoRecord is ON
                    frame = getframe( obj.hFigure );                       % Get the current frame of the figure
                    writeVideo( writerObj, frame );                        % Writing it to the mp4 file/
                else                                                       % If videoRecord is OFF
                    drawnow                                                % Just simply show at Figure
                end

            end   

            if isVidRecord
                close( writerObj );
            end         

        end
        
        function goto( obj, idx )
            % Go to the step of the following idx. 
            set( obj.hTitle,'String', sprintf( '[Time] %5.3f (s)  x%2.1f', obj.tVec( idx ),  obj.vidRate  ) )
        
            % Update Graphics
            % We'll assume that all graphics are 3D, since we can just see in the 2D plane for 2D plot
            for i = 1 : length( obj.hAxes )
               
                if isempty( obj.hGraphicObjs{ i } )
                    continue     % Go to next iteration
                end
                
                for j = 1 : length( obj.hGraphicObjs{ i } )                % Iterating along each graphs.
                    
                    for attr = obj.graphicObjs{ i }( j ).gAttrUpdateList   % Getting the list of values that should be updated
                            
                        if obj.graphicObjs{ i }( j ).type == "ellipsoid"
                            set( obj.hGraphicObjs{ i }( j ), attr, obj.graphicObjs{ i }( j ).( attr )( :, :, idx ) );   % Since ellipse is a 3D object. 
                            
                        else
                            set( obj.hGraphicObjs{ i }( j ), attr, obj.graphicObjs{ i }( j ).( attr )( :, idx ) );
                        end
                            
                    end
                    
                end
                 
            end
           
            
            % Update the zoomed-in view's xlim, ylim and zlim
            iidx = find( obj.isZoomed );  % Find the index that should be changed. 
            if iidx ~= 0
                set( obj.hAxes{ iidx },  'XLim',  [ -obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).XData( idx ), obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).XData( idx ) ] , ...         
                                         'YLim',  [ -obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).YData( idx ), obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).YData( idx ) ] , ...    
                                         'ZLim',  [ -obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).ZData( idx ), obj.zoomSize + obj.graphicObjs{ iidx }( obj.zoomIdx ).ZData( idx ) ] )  
            end
           
        end
           
        
        function addTrackingPlots( obj, idx, marker )
            %addZoomWindow: Add a Zoom-in view plot of the main axes plot
            % [INPUTS]            
            %   (1) idx [integer]
            %       -  2 (Lower-right), 3 (Upper-right)            
            %   (2) tdata, ydata [float array]
            %       -  The data array for the tracker `
            
            obj.adjustFigures( idx )                     

            plot( obj.hAxes{ idx }, marker.XData, marker.YData, 'color', marker.MarkerEdgeColor )
            
            % Add the tracking markers.               
            obj.addGraphicObject( idx, marker );
            
            % Add Static plot for the graph
            
            
        end
        
        function addZoomWindow( obj, idx, whichMarker, size )
            %addZoomWindow: Add a Zoom-in view plot of the main axes plot
            % [INPUTS]            
            %   (1) idx [integer]
            %       -  1 (Main), 2 (Lower-right), 3 (Upper-right)            
            %   (2) whichMarker [integer/string]
            %       -  The Marker that we want to focus
            %   (3) size 
            %       -  The size of the zoom-in window.
            
            obj.adjustFigures( idx )         
            
            for i = 1 : length( obj.graphicObjs{ 1 } )
                
                if  strcmp( obj.graphicObjs{ 1 }(i).name, whichMarker )
                   idxM = i; 
                end
            end
                
            % Copy the graphic objects from main to the plot
            obj.isZoomed( idx ) = true;
            obj.zoomIdx  = idxM; 
            obj.zoomSize = size;
            
            % Copy and overwrite all the graphic objects from main to side plot
            % Need to use "copyobj" for the plot
            obj.hGraphicObjs{ idx } = copyobj( obj.hGraphicObjs{ 1 }, obj.hAxes{ idx } );             
            obj.graphicObjs{  idx } = obj.graphicObjs{ 1 };
            
            
        end
        
    end

end


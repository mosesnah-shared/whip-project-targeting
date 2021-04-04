classdef my2DOFRobot < handle
% % =============================================================== %
%   [DESCRIPTION]
%       Class for generating an 2-DOF planar robot 
%
%   [CREATED BY]  Moses Nah
%   [EMAIL]       mosesnah@mit.edu
%   [AFFILIATION] Massachusetts Institute of Technology (MIT)
%   [MADE AT]     18-October-2020    
% % =============================================================== %    
    properties ( SetAccess = private )
        
    end
        
    properties ( SetAccess = public )

                        % [Symbolic values]
                        % [DESCRIPTIONS]                                    [SIZE]
        t;              % Time                                             (1-by-1) 
        g;              % Gravity                                          (1-by-1)
        q;              % Joint angles                                     (1-by-n) 
        dq;             % Joint velocities                                 (1-by-n) 
        ddq;            % Joint accelerations                              (1-by-n) 
        
        dqr;            % Reference Joint velocities                       (1-by-n) 
        ddqr;           % Reference Joint accelerations                    (1-by-n)         
        
        L;              % Length of links                                  (1-by-2) 
        Lc;             % Length from proximal joint to C.O.M.             (1-by-2) 
        M;              % Mass of links                                    (1-by-2) 
        I;              % Inertia of links, must be diagonal               (2-by-3) 
        
                        % se( 3 ) Matrix for the forward kinematics 
        T_arr;          % Array to fill T01, T02, ... T0n
        
        idx_limb        % The index list which makes the frame vs. limb corresponding.
                        % For instance, 1st frame denotes the first  limb. 
                        %               2nd frame denotes the second limb.
                        % hence the idx_limb = [1,2]
                        % The idx_limb should be initialized under obj.initialize() 
                        % M, C Matrix and G, tau vector of the manipulator equation of the upper-limb model 
        M_mat;
        C_mat;
        G_mat;
        
        nDOF;
       
        
        r;              % r is arguments for varargin
    
    end
    
    methods (Static)

        function vs = body2spatial( Rsb, vb )
        % ================================================================             
        % Changing the body frame vector to spatial coordinate
        % ================================================================             
        % [INPUT]
        %    (1) Rsb - Rotation so(3) matrix, which transforms the body frame vector to spatial frame vector
        %    (2) vb  - Vector described in body (b) frame
        % ================================================================ 
        % [OUTPUT]
        %    (1) vs  - Vector described in spatial (s) frame
        % ================================================================                 
             
             vs = Rsb * vb;
        end 
        
        
        function [w, v] = se3_2_vec( mat )
        % ================================================================             
        % Changing the se3 matrix to vector format
        % The se3 matrix is in the following format [[w], v; 0,0,0,1] 
        % ================================================================             
        % [INPUT]
        %    (1) mat - 4-by-4 se(3) matrix
        % ================================================================ 
        % [OUTPUT]
        %    (1) v  - Vector form of the se(3) matrix
        % ================================================================    
        
             w = [ -mat( 2,3 ); mat( 1,3 ); -mat( 1,2 ) ];
             v = mat( 1:3, 4 );
        end         
        
        function T = se3( q, L, func )
        % ================================================================
        % Returning the 4-by-4 se(3) matrix for forward kinematics
        % Assume that we only consider rotational joints
        % [INPUT]
        %    (1) q (scalar) the amount of rotation
        %    (2) L (3-by-1) The xyz position w.r.t. the frame 
        %    (3) func       rotx, roty, rotz
        % ================================================================ 
        % [OUTPUT]
        %    (1) T se(3) matrix, [[Rot(q)], v; 0,0,0,1] 
        % ================================================================
        
            if ~isa( func, 'function_handle' )
                error(  "3rd argument should be function handle of rotx, roty and rotz" )
            end

            name = functions( func ).function;
            
            if ~( strcmp( name, 'rotx' ) || strcmp( name, 'roty'  ) || strcmp( name, 'rotz' ) ) 
                error(  "3rd argument should be function handle of rotx, roty and rotz" )
            end

            T = [    func( q ),  L; ...
                  zeros( 1,3 ),  1 ];
        
        end
        
        
        function J = myJacobian( x1, x2 )
        % ================================================================
        % Getting the Jacobian from x2 to x1.
        % Meaning, dx1 = J dx2
        % [TODO] If possible, we should make this as a single line or so.
        % ================================================================         
        % [INPUT]
        %    (1) dx1, which satisfies dx1 = J dx2
        %    (2) dx2, which satisfies dx1 = J dx2
        % ================================================================ 
        % [OUTPUT]
        %    (1) J which satisfies dx1 = J dx2
        % ================================================================            
          
            J = sym( 'J', [ length( x1 ), length( x2 ) ] );

            for i = 1 : length( x1 )                                           
                for j = 1 : length( x2 )                                       

                    J( i, j ) = functionalDerivative( x1( i ), x2( j ) );

                end
            end

        end
        
        function A = getCoefficient( x1, x2 )
        % ================================================================
        % Get or extract the coefficients of dx2s from dx1 vector
        % This method becomes handy when we want to calculate the Y matrix,
        % which is the dynamic regressor matrix for the adaptive controller
        % ================================================================         
        % [INPUT]
        %    (1) dx1: dx1 = Adx2 form
        %    (2) dx2: dx1 = Adx2 form
        % ================================================================ 
        % [OUTPUT]
        %    (1) A: dx1 = Adx2 form
        % ================================================================          
        
            A = sym( 'A', [ length( x1 ), length( x2 ) ] );

            for i = 1 : length( x1 )                                 
                for j = 1 : length( x2 )                             

                    [ tmpc, tmpt ] = coeffs( x1( i ), x2( j ) );    

                    % IF the coefficients (tmpc) corresponding to dq(3) is empty, put zero
                    tmp = tmpc( tmpt == x2( j ) );
                    
                    if( isempty( tmp ) )
                        A( i, j ) = 0; 
                    else    
                        A( i, j ) = tmp;
                    end

                end
            end
        end   

    end
    % ========================================
    % End of Static Methods
    % ========================================
    
    % ========================================
    % Start of General Methods
    % ========================================    
    
    methods
        
        function obj = my2DOFRobot( varargin ) 
        % ================================================================
        % The constructor of the my2DOFRobot
        % [MOSES] 2020.12.05
        %      We will stick with 2 limb segments for quite a while
        % ================================================================         
        % [INPUT]
        %    (1) varargin
        %        - TO_BE_EXPLAINED SOON
        % ================================================================ 
            
            syms t positive                                                % time  variable - independent variable.
            
            obj.nDOF  = 2;                                                 % The degrees of freedom of the robot.   
            obj.t     = t;                                                 % saving the time variable as the member variable
            obj.g     = sym('g', 'real' );                                 % The gravity of the system, we will set the gravity direction as -z.
            
            obj.idx_limb = [ 1, 2 ];                                               
            
            
        end
        
        function initialize( obj )
            % Initialization of the class + parameters 
            
            
            % There is yet no way to generate array of symfun, for our case, the q(t) and dq(t) vectors
            % [REF] https://www.mathworks.com/matlabcentral/answers/391756-how-to-declare-time-dependent-symbolic-array
            
            tmp = { 'q', 'dq', 'ddq' };
            
            for i = 1 : length( tmp )  % Iterate along q, dq, ddq  
                
                tmp2 = sym( zeros( 1, obj.nDOF ) );                        % Define an empty array
                
                for j = 1 : obj.nDOF 
                    var_str   = strcat( tmp{ i }, num2str( j ), '(t)' );   % Builds the symbolic variable string, e.g., q1, q2, q3...
                    tmp2( j ) = str2sym( var_str );                        % store the variable in the array
                end
                
                obj.( tmp{ i } ) = tmp2;                                   % Replacement of q (sym) to q(t) (symfunc)
                
            end
            
            % Defining geometrical/inertial symbolic values 
            syms I1xx I1yy I1zz I2xx I2yy I2zz positive
            
            obj.L   = sym( 'L' ,[1,2], 'positive' );                       % Length of limb segment
            obj.Lc  = sym( 'Lc',[1,2], 'positive' );                       % Length from proximal joint to COM
            obj.M   = sym( 'M' ,[1,2], 'positive' );                       % Mass   of limb segment
            obj.I   = [ I1xx, I1yy, I1zz; ...
                        I2xx, I2yy, I2zz];                                 % Inertia w.r.t. C.o.M.
           
            % Initializing/seeting the frames' SE(3) matrices 
            obj.setFrameSE3(  )                         
                    
        end
        
        function setFrameSE3( obj )
            % Setting the SE(3) information of the frames. 
            % THIS PROCESS IS NECESSARY FOR ALL CALCULATION
            % Defining the se(3) matrix for each coordinate transformation        
            % For details of the frame coordinate information, please refer to the following paper 
            % [REF] Nah, Moses C. Dynamic primitives facilitate manipulating a whip. Diss. Massachusetts Institute of Technology, 2020.
            % Fig. 3.4.
            T01 = obj.se3( -obj.q( 1 ), zeros( 3, 1 )         , @roty );           
            T12 = obj.se3( -obj.q( 2 ), [ 0; 0; -obj.L( 1 ) ] , @roty );
            
            % [WARNING!!!]
            % We need to fill in the T_arr before we conduct .getM, .getC and .getG methods
            obj.T_arr = { T01, T01 * T12 };                                % This is the T array (SE(3) matrix) for the "frame" coordinate. For finding the C.O.M. frame we need additional process.
            
        end    
            
        function [M, C, G] = deriveManipulatorEquation( obj )
            % Derive the manipulator equation:
            % tau = Mq'' + Cq' + G
            
            M = obj.getM( );
            C = obj.getC( );
            G = obj.getG( );

        end

        function FK = forwardKinematics( obj, idx, L )
        % ================================================================             
        % [INPUT]
        %    (1) idx, 1 is the first link, 2 is the 2nd link
        %    (2) L is the length of the point where the jacobian should be calculated. 
        %    (3) q is the relative angle array of the 2DOF robot. 
        %        Can be done in array, which is 2-by-N, N is the time-series data
        % ================================================================ 
        % [OUTPUT]
        %    (1) FK, the position of the given point
        % ================================================================    
        
             %   To neglect the 4th element, 3-by-4 matrix multiplication
             % obj.idx_limb returns the frame number (or the number of Tarr ) which corresponds to the i-th limb segment.             
             FK = [ eye(3), zeros(3,1) ] * obj.T_arr{ obj.idx_limb( idx ) } * [ L; 1 ];
                
        end        
        
        
        function M = getM( obj )
        % Calculating the mass matrix of the model 
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-complete.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================             
        
            % Initializing the mass matrix
            M = 0;  
            
            for i = 1 : length( obj.Lc ) % Iterating along the number of c.o.ms
                

                % obj.idx_limb returns the frame number (or the number of Tarr ) which corresponds to the i-th limb segment.
                % tmp corresponds to the se(3) matrix for the C.O.M. of each limb segments. 
                %                                                                   IGNORE! rotz is just a dummy function parameter, since q is 0
                tmp = obj.T_arr{ obj.idx_limb( i ) } * obj.se3( 0, [ 0; 0; -obj.Lc( i ) ], @rotz );
                
                % Calculation of body velocity is necessary for getting the generalized mass matrix
                % inv(T) * d(T) = V_b, where V_b is 4-by-4 matrix                
                Vb = simplify( tmp \ diff( tmp, obj.t ) );  
                Vb = subs( Vb, diff( obj.q, obj.t ), obj.dq );             % Simple substitution
                                                                           % dq will be shown as diff( q, t ) since q is a symfun. 
                                                                           % Substituting them to obj.q --> obj.dq
                
                % The result of Vb matrix is [[w], v; 0,0,0,1], where w and v are angular and translational velocity, respectively.                      
                [ w, v ] = obj.se3_2_vec( Vb );                            % se(3) to w, v vector transformation
                
                % Calculating the body jacobian 
                J = obj.myJacobian( [v; w], obj.dq );
                
                % Generalized mass/inertia matrix
                tmp = diag( [ obj.M( i ),   obj.M( i ),   obj.M( i ), ...
                            obj.I( i,1 ), obj.I( i,2 ), obj.I( i,3 )    ]);
            
                M = M + simplify( J.' * tmp * J );                     % .' If you don't add dot before the transpose notation, the transpose will return conjugate
                
            end
               
            obj.M_mat = M;    

        end


        function C = getC( obj )
        % Calculating the Coriolis matrix of the manipulator
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================                  
        
            if isempty( obj.M_mat )
                obj.getM( );                                               % Fill in the mass matrix if empty. 
            end

            % Calculating the coriolis term matrix of the model 
            n = length( obj.q );
            C = sym( 'C', [ n, n ] );

            % C matrix is determined by the Christoffel symbols:
            for i = 1 : n
                for j = 1 : n
                    tmp = 0;
                    for k = 1 : n
                        tmp1 =   1 / 2 * functionalDerivative( obj.M_mat( i, j ), obj.q( k ) ) ...
                               + 1 / 2 * functionalDerivative( obj.M_mat( i, k ), obj.q( j ) ) ...
                               - 1 / 2 * functionalDerivative( obj.M_mat( k, j ), obj.q( i ) );
                        tmp1 = tmp1 * obj.dq( k );
                        tmp  = tmp + tmp1;
                    end
                    C( i,j ) = simplify( tmp );
                end
            end        
        
            obj.C_mat = C;    
        end
        
        function G = getG( obj )
        % Calculating the "gravity-vector" of the manipulator
        % ================================================================             
        % [REF1] https://www.cds.caltech.edu/~murray/books/MLS/pdf/mls94-manipdyn_v1_2.pdf
        % [REF2] http://robotics.snu.ac.kr/edX/2014-2/lecturenotes/SNUx_Chapter_6.pdf
        % ================================================================                  

            G = sym( 'G', [ 1, length( obj.q ) ] );
        
            V = 0;
            
            for i = 1 : length( obj.L )
                pc = obj.forwardKinematics( i, [0; 0; -obj.Lc( i )] );
                V  = V + obj.M( i ) * obj.g * pc( end );                   % Calculating the Gravitational Energy
            end
            
            for i = 1 : length( obj.q )
                G( i ) = simplify( functionalDerivative( V, obj.q( i ) ) );
            end
            
            obj.G_mat = G;
        end        
        
        
        function [tau, Y, a] = findManipulatorRegressor( obj )
            % This method is for calculating the  Y matrix and a vector, which will substitute
            % Mq'' + Cq' + G = Ya
            % Note that ddqr, dqr should be defined since sliding control is involved in the controller
            % [REF] Slotine, Jean-Jacques E., and Weiping Li. "On the adaptive control of robot manipulators." The international journal of robotics research 6.3 (1987): 49-59.
            
            
            obj.ddqr = sym( 'ddqr', [ 1, obj.nDOF ] );
            obj.dqr  = sym(  'dqr', [ 1, obj.nDOF ] );
            
            tau = obj.M_mat * obj.ddqr.' + ...
                  obj.C_mat * obj.dqr.'  + ...
                  obj.G_mat.';            
            
            
            % The possible a vector that might be included in the manipulator equation.
            % Later, it will be trimmed down
            % [TODO] we can also add a as the "kinematic" jacobian elements, but not yet included.
            
            % All the possible forms of the a vector is as follows:
            % M * Lc * L, M * L^2, M * Lc^2, M * g * L, M * g * Lc
            % This can simply done with the kronecker multiplication
            avec = [ reshape( obj.I, 1, [ ] ),                                      ...   Flatten the I matrix, 
                     kron( obj.M,   kron( [ obj.L, obj.Lc ], [ obj.L, obj.Lc ] ) ), ...   Calculating everything with M * Lc * L, M * L^2, M * Lc^2
                     kron( obj.g * obj.M, [ obj.L, obj.Lc  ] ) ];

                   
            asym = sym( 'a', [ 1, length( avec ) ] );  

            % First, we need to substitue the tau function with the a values  
            tau_sub = sym( 'tau_sub', [ obj.nDOF,1] );

            % [Preprocessing]
            % Changing all the unknown terms to a variables.
            for i = 1 : obj.nDOF

                tmp = tau( i );

                for j = 1 : length( avec ) 

                    tmp = subs( expand( tmp ), avec( j ), asym( j ) );

                end

                tau_sub( i ) = tmp;
            end       
                   
         
            % [Preprocessing]
            % To find Y matrix, we simply need to find the matrix which 
            % consists of the coefficients of the a vectors
            Y = obj.getCoefficient( tau_sub, asym );
            
            % [Preprocessing]            
            % Clean up the columns filled with 0, we don't simply need them!
            % Get the index of it and clean-up!
            idx = ~all( Y == 0 );
            Y   =   simplify( Y( :,idx ) );
            a   =   avec(  idx );
            
            
            % After we tease out the Y Matrix, find the columns which are same. 
            i = 1; 
            while i ~= length( a )
                
                ismatch = false;                                           % initializing the boolean ismatch to check whether 
                                                                           % The Y Columns are the same.
                
                for j = i + 1 : length( a )                                % Iterating along the a vector
                    
                    if isequal( Y( :, i ), Y( :, j ) )                     % If column is equal, add the a values and erase the part
                        
                        a( i ) = a( i ) + a( j );                          % Add the a components and
                        a( j ) = []; Y( :, j ) = [];                       % Simply delete the repeated elements
                        
                        ismatch = true;
                        break
                        
                    end
                    
                end
                
                if ~ismatch                                                % If no match, then preceed forward
                   i = i + 1; 
                end
                
            end
            
        end
        

    end

        
end



classdef standard < matlab.System ... %#codegen
    ... & matlab.system.mixin.FiniteSource ...
    & matlab.system.mixin.CustomIcon ...
    & matlab.system.mixin.Propagates ...
    ... & matlab.system.mixin.SampleTime ...
    ... & matlab.system.mixin.Nondirect
    %% STANDARD Standard forward kinematics of a CDPR
    %
    %   P = STANDARD(L) estimates the pose P based on cable lenghts L.
    %
    %   Inputs:
    %
    %   L               1xM vector of absolute cable lengths for the given pose
    %                   P.
    %
    %   Outputs:
    %
    %   P               1xNP vector of pose uniquely defining the CDPR platform
    %                   pose. Pose P must be formatted as following depending on
    %                   the motion pattern in use:
    %                   1T:    P = [X]
    %                   2T:    P = [X, Y]
    %                   3T:    P = [X, Y, Z]
    %                   1R2T:  P = [X, Y, R11, R12, R21, R22]
    %                   2R3T:  P = [X, Y, Z, R11, R12, R13, R21, R22, R23, R31, R32, R33]
    %                   32R3T: P = [X, Y, Z, R11, R12, R13, R21, R22, R23, R31, R32, R33]

  
  
  %% PUBLIC, NON-TUNABLE LOGICALS
  properties ( Nontunable, Logical )
    
  end
  
  
  %% PUBLIC, NON-TUNABLE INTEGERS
  properties ( Nontunable, PositiveInteger )
    
  end
  
  
  %% PUBLIC NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % CDPR motion pattern
    MotionPattern = '3R3T';
    
    % Winch/pulley locations
    Frame = zeros(3, 1);
    
    % Platform anchors
    Platform = zeros(3, 1);
    
    % Additional cable length offset
    CableOffset = zeros(1, 1);
    
    % Optimization Method
    OptimizationMethod = 'lsqnonlin';
    
    % Maximum number of function evaluations allowed, a positive integer
    % See Tolerances and Stopping Criteria and Iterations and Function Counts.
    MaxFunctionEvaluations = 100;
    
    % Maximum number of iterations allowed, a positive integer
    % See Tolerances and Stopping Criteria and Iterations and Function Counts.
    MaxIterations = 400;
    
    % Termination tolerance on the function value, a positive scalar.
    % See Tolerances and Stopping Criteria.
    FunctionTolerance = 1e-8;
    
    % Termination tolerance on the first-order optimality, a positive scalar.
    % See First-Order Optimality Measure.
    OptimalityTolerance = 1e-8;
    
    % Termination tolerance on x, a positive scalar.
    % See Tolerances and Stopping Criteria.
    StepTolerance = 1e-8
    
  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )
    
  end
  
  
  %% PROTECTED NON-TUNABLE PROPERTIES
  properties ( Nontunable, Access = protected )
    
    % Lower bounds for estimator
    LowerBounds = [];
    
    % Upper bounds for estimator
    UpperBounds = [];
    
  end

  
  %% CONSTANT HIDDEN PROPERTIES
  properties ( Constant, Hidden )
    
    % Supported motion patterns
    MotionPatternSet = matlab.system.StringSet({'1T', '2T', '3T', '1R2T', '2R3T', '3R3T'});
    
    % Possible optimization methods
    OptimizationMethodSet = matlab.system.StringSet({'newton-raphson', 'lsqnonlin', 'fmincon', 'fsolve'});
    
  end
  
  
  %% PROTECTED PROPERTIES
  properties ( Access = protected )
    
    % Inverse kinematics object
    ik@ik.standard
    
    % Structure matrix object
    sm@sm.structurematrix
    
    % Options for the solver, need create them only once at setup
    SolverOptions = {};
    
    % Pose from the previous optimization used as initial estimate for future
    % poses
    PreviousEstimate
    
  end
  
  
  %% PROTECTED DEPENDENT PROPERTIES
  properties ( Access = protected )
    
    % Number of cables
    NCables
    
    % Number of pose entries
    NPose
    
    % Number of degrees of freedom
    NDoF
    NDoF_Linear
    NDoF_Rotational
    
    % Boundaries of pose estimate for estimator
    Bounds_Lower
    Bounds_Upper
    
  end
  
  
  %% PRIVATE PROPERTIES
  properties ( Access = private )
    
  end
  
  
  
  %% CONSTRUCTOR
  methods
    
    function this = standard(varargin)
      %% STANDARD
      % Support name-value pair arguments when constructing object
      
      
      % Now set all properties
      setProperties(this, nargin, varargin{:})
      
    end
    
  end
  
  
  %% SETTERS
  methods
    
  end
  
  
  
  %% GETTERS
  methods
    
  end
  

  
  %% ALGORITHM
  methods ( Access = protected )
    
    function setupImpl(this)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
      % Number of cables
      this.NCables = size(this.Frame, 2);
      
      % Number of linear and rotational DoF, number of pose entries
      switch this.MotionPattern
        case '1T'
          this.NDoF_Linear = 1;
          this.NDoF_Rotational = 0;
          this.NPose = 1 + 0;
          
        case '2T'
          this.NDoF_Linear = 2;
          this.NDoF_Rotational = 0;
          this.NPose = 2 + 0;
          
        case '3T'
          this.NDoF_Linear = 3;
          this.NDoF_Rotational = 0;
          this.NPose = 3 + 0;
          
        case '1R2T'
          this.NDoF_Linear = 2;
          this.NDoF_Rotational = 1;
          this.NPose = 3 + 4;
          
        case '2R3T'
          this.NDoF_Linear = 3;
          this.NDoF_Rotational = 2;
          this.NPose = 3 + 9;
          
%         case '3R3T'
        otherwise
          this.NDoF_Linear = 3;
          this.NDoF_Rotational = 3;
          this.NPose = 3 + 9;
          
      end
      % Sum  of all DOF
      this.NDoF = this.NDoF_Linear + this.NDoF_Rotational;
      
      % Set an initial previous pose estimate
      this.PreviousEstimate = zeros(1, this.NPose);
      
%       % Create an initial solver options set so it doesn't have to be created on
%       % every step
%       if ~strcmp(this.OptimizationMethod, 'newton-raphson')
%         this.SolverOptions = optimoptions(this.OptimizationMethod ...
%           , 'MaxFunctionEvaluations', this.MaxFunctionEvaluations ...
%           , 'MaxIterations', this.MaxIterations ...
%           , 'FunctionTolerance', this.FunctionTolerance ...
%           , 'OptimalityTolerance', this.OptimalityTolerance ...
%           , 'StepTolerance', this.StepTolerance ...
%           , 'SpecifyObjectiveGradient', true ...
%           , 'Display', 'off' ...
%           , 'ScaleProblem', 'jacobian' ...
%         );
%       else
        this.SolverOptions = struct( ...
            'MaxFunctionEvaluations', this.MaxFunctionEvaluations ...
          , 'MaxIterations', this.MaxIterations ...
          , 'FunctionTolerance', this.FunctionTolerance ...
          , 'StepTolerance', this.StepTolerance ...
          , 'Display', 'off' ...
          , 'Algorithm', 'levenberg-marquardt' ...
        );
%       end
      
%       % Add solver specific arguments
%       switch this.OptimizationMethod
%         case 'newton-raphson'
%           
%         case 'lsqnonlin'
%           this.SolverOptions = optimoptions(this.SolverOptions ...
%             , 'Algorithm', 'trust-region-reflective' ...
%             ... , 'Algorithm', 'levenberg-marquardt' ...
%           );
%         
%         case 'fsolve'
%           this.SolverOptions = optimoptions(this.SolverOptions ...
%             , 'Algorithm', 'trust-region-reflective' ...
%             ..., 'Algorithm', 'levenberg-marquardt' ...
%           );
%           
%         case 'fmincon'
%           this.SolverOptions = optimoptions(this.SolverOptions ...
%             , 'Algorithm', 'sqp' ...
%           );
%       end
      
      % Update cable offset
      if isscalar(this.CableOffset)
        this.CableOffset = this.CableOffset .* ones(1, this.NCables);
      end
      
      % Make IK and SM objects
      switch this.MotionPattern
        case '1T'
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern);
        
        case '2T'
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern);
        
        case '3T'
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern);
        
        case '1R2T'
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'Platform', this.Platform, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern, 'Platform', this.Platform);
        
        case '2R3T'
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'Platform', this.Platform, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern, 'Platform', this.Platform);
        
%         case '3R3T'
        otherwise
          this.ik = ik.standard('MotionPattern', this.MotionPattern, 'Frame', this.Frame, 'Platform', this.Platform, 'CableOffset', this.CableOffset, 'UnitVectors', true);
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern, 'Platform', this.Platform);
      end
      
      % Calculate lower and upper bounds
      switch this.MotionPattern
        case '1T'
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = [];
          Rmax = [];
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(sign(rmin) == sign(abs(rmin))) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(sign(-rmax) == sign(abs(rmax))) = Inf;
          
        case '2T'
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = [];
          Rmax = [];
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(sign(rmin) == sign(abs(rmin))) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(sign(-rmax) == sign(abs(rmax))) = Inf;
          
        case '3T'
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = [];
          Rmax = [];
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(sign(rmin) == sign(abs(rmin))) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(sign(-rmax) == sign(abs(rmax))) = Inf;
          
        case '1R2T'
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = -pi;
          Rmax = pi;
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(rmin == rmax) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(rmax == rmin) = Inf;
          
        case '2R3T'
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = [-pi, -pi];
          Rmax = [pi, pi];
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(sign(rmin) == sign(abs(rmin))) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(sign(-rmax) == sign(abs(rmax))) = Inf;
          
%         case '3R3T'
        otherwise
          rmin = transpose(min(this.Frame, [], 2));
          rmax = transpose(max(this.Frame, [], 2));
          Rmin = [-pi, -pi, -pi];
          Rmax = [pi, pi, pi];
          % If any minimum position equals any maximum position, we set this
          % minimum to -Inf
          rmin(sign(rmin) == sign(abs(rmin))) = -Inf;
          % If any maximum position equals any minimum position, we set this
          % minimum to Inf
          rmax(sign(-rmax) == sign(abs(rmax))) = Inf;
          
      end
      
      % Lower bounds and upper bounds for the pose
      this.LowerBounds = [rmin, Rmin];
      this.UpperBounds = [rmax, Rmax];
      
    end
    
    
    function [p, e] = stepImpl(this, l)
      %% STEPIMPL
      % Implement algorithm. Calculate y as a function of input u and discrete
      % states.
      
      
      % Different optimization methods require different types and orders of
      % arguments
      switch this.OptimizationMethod
        case 'newton-raphson'
          o = newton_raphson(this, l);
        case 'lsqnonlin'
          % Optimize away
          o = lsqnonlin(@(o) lsqnonlin_target(this, o, l), pose_estimate(this, l), this.LowerBounds, this.UpperBounds, this.SolverOptions);
        case 'fsolve'
          % Optimize away
          o = fsolve(@(o) lsqnonlin_target(this, o, l), pose_estimate(this, l), this.SolverOptions);
        case 'fmincon'
          % Optimize away
          o = fmincon(@(o) fmincon_target(this, o, l), pose_estimate(this, l), [], [], [], [], this.LowerBounds, this.UpperBounds, [], this.SolverOptions);
      end
      
      % And convert the optimziation state to a pose to return data properly
      eo = pose_estimate(this, l);
      p = optimstate2pose(this, o);
      e = optimstate2pose(this, eo);
      
      % Update previous pose value
      this.PreviousEstimate = p;
      
    end
    
    
    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties
      
      
      % Calculate initial pose estimate
      switch this.MotionPattern
        case '1T' % [x]
          o0 = 0;
        
        case '2T' % [x,z]
          o0 = [0,0];
        
        case '3T' % [x,y,z]
          o0 = [0,0,0];
        
        case '1R2T' % [x,z, c]
          o0 = [0,0, 0];
        
        case '2R3T' % [x,y,z, a,b,c]
          o0 = [0,0,0, 0,0,0];
        
%         case '3R3T' % [x,y,z, a,b,c]
        otherwise
          o0 = [0,0,0, 0,0,0];
      
      end
      this.PreviousEstimate = o0;
      
    end
    
    
    function releaseImpl(this)
      %% RELEASEIMPL
      % Release resources, such as file handles
      
      
    end
    
  end
  
  
  
  %% OPTIMIZATION METHODS
  methods ( Access = protected )
    
    function [f, J] = lsqnonlin_target(this, o, l)
      %% LSQNONLIN_TARGET
      %
      %   F = LSQNONLIN_TARGET(O, L) calculates the vector of errors between the
      %   cable lengths from the state of optimization O and the given cable
      %   lengths L.
      %
      %   [F, J] = LSQNONLIN_TARGET(O, L) also returns the Jacobian J of the
      %   target function F evaluated at O.
      %
      %   Inputs:
      %
      %   O             Kx1 vector of the to-be-estimated pose.
      %
      %   L             Mx1 vector of the target cable lengths to estimate pose
      %                 for.
      %
      %   Outputs:
      %
      %   F             Mx1 vector of difference between the estimated pose's
      %                 cable length and the given cable lengths.
      %
      %   J             MxK matrix of the Jacobian of the residual
      
      
      % Optimization state to pose
      p = optimstate2pose(this, o);
      
      % Use the state of optimization to calculate its
      [lo, uo] = this.ik(p);
      
      % Calculate the error
      f = lo(:) - l(:);
      
      % Return Jacobian?
      if nargout > 1
          % Dispatch Jacobian to the structure matrix object
          J = -transpose(this.sm(p, uo));
      
      end
      
    end
    
    
    function [r, J] = fmincon_target(this, l, o)
      %% FMINCON_TARGET 
      %
      %   F = FMINCON_TARGET(O, L) calculates the residual of the vector of
      %   errors between the cable lengths from the state of optimization O and
      %   the given cable lengths L.
      %
      %   [F, J] = FMINCON_TARGET(O, L) also returns the Jacobian J of the
      %   target function F evaluated at O.
      %
      %   Inputs:
      %
      %   O             Kx1 vector of the to-be-estimated pose.
      %
      %   L             Mx1 vector of the target cable lengths to estimate pose
      %                 for.
      %
      %   Outputs:
      %
      %   F             Mx1 vector of difference between the estimated pose's
      %                 cable length and the given cable lengths.
      %
      %   J             MxK matrix of the Jacobian of the estimation.
      
      
      % Convert optimization vector to a correct pose
      p = optimstate2pose(this, o);
      
      % Get cable lengths for the estimated pose
      [lo, uo] = this.ik(p);
      
      % Error
      e = lo(:) - l(:);
      
      % Residual is || e || ^2;
      r = sum(e .^ 2);
      
      % Build the Jacobian
      if nargout > 1
          % Dispatch Jacobian to the structure matrix object
          J = -transpose(this.sm(p, uo));
      end
      
    end
    
    
    function p = optimstate2pose(this, o)
      %% OPTIMSTATE2POSE
      %
      %   P = OPTIMSTATE2POSE(O) converts the optimization state O to a valid
      %   pose list entry.
      %
      %   Inputs:
      %
      %   O             1xK vector of the to-be-estimated pose
      %
      %   Outputs:
      %
      %   P             1xP vector of the pose list entry
      
      
      switch this.MotionPattern
        case '1T'
          p = o(1);
        
        case '2T'
          p = [o(1), o(2)];
        
        case '3T'
          p = [o(1), o(2), o(3)];
        
        case '1R2T'
          % Convert orientation to rotation matrix entries
          so = sin(o(3));
          co = cos(o(3));
          % Build pose
          p = [o(1), o(2), co, -so, so, co];
          
        case '2R3T'
          % Convert orientation to a rotation matrix
          R = eul2rotm([o(4), o(5), o(6)], 'ZYX');
          % Build pose
          p = [o(1), o(2), o(3), R([1, 4, 7, 2, 5, 8, 3, 6, 9])];
        
%         case '3R3T'
        otherwise
          % Convert orientation to a rotation matrix
          R = eul2rotm([o(4), o(5), o(6)], 'ZYX');
          % Build pose
          p = [o(1), o(2), o(3), R([1, 4, 7, 2, 5, 8, 3, 6, 9])];
          
      end
      
    end
    
    
    function e = pose_estimate(this, l)
      %% POSE_ESTIMATE estimate initial pose for forward kinematics
      % 
      %   E = POSE_ESTIMATE(L) calculates the center of the bounding box of the
      %   given pulley positions and cable attachments as an initial guess for
      %   the forward kinematics.
      %   
      %   Inputs:
      %   
      %   L             1xM vector of cable lengths to use for calculation.
      %
      %   Outputs:
      % 
      %   E             1xN vector representing the initial pose of the system
      %                 where N matches the pose length of the current motion
      %                 pattern.
      
     
      switch this.MotionPattern
          case '1T'
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l, this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l, this.NDoF_Linear, 1), [], 2);
            
            % Use the center of the bounding box as an initial estimate
            e = (rmin + rmax) / 2;
            
          case '2T'
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l, this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l, this.NDoF_Linear, 1), [], 2);
            
            % Use the center of the bounding box as an initial estimate
            e = (rmin + rmax) / 2;
            
          case '3T'
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l, this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l, this.NDoF_Linear, 1), [], 2);
            
            % Use the center of the bounding box as an initial estimate
            e = (rmin + rmax) / 2;
            
          case '1R2T'
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            
            % Init estimate
            e = zeros(1, 3);
            % Use the center of the bounding box as an initial estimate
            e(1:2) = (rmin + rmax) / 2;
            % Small orientational offset
            e(3) = 1e5*eps;
            
          case '2R3T'
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            
            % Init estimate
            e = zeros(1, 6);
            % Use the center of the bounding box as an initial estimate
            e(1:3) = (rmin + rmax) / 2;
            % Small orientational offset
            e(4:6) = 1e5*eps;
            
%           case '3R3T'
          otherwise
            % Calculate bounding box around the whole frame
            rmin = max(this.Frame - repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            rmax = min(this.Frame + repmat(l + sqrt(sum(this.Platform.^2, 1)), this.NDoF_Linear, 1), [], 2);
            
            % Init estimate
            e = zeros(1, 6);
            % Use the center of the bounding box as an initial estimate
            e(1:3) = (rmin + rmax) / 2;
            % Small orientational offset
            e(4:6) = 1e5*eps;
      end
    
    end
    
    
    function o = newton_raphson(this, l)
      %% NEWTON_RAPHSON performs simple Newton-Raphson iteration on the forward kinematics
      
      
      % Init loop variables
      kiter = 0;
      converged = false;
      % Maximum halvings of damped Newton-Raphson factor
      maxhalfs = 1/1024;
      % Scaling factor accounting for numerical inaccuracies when comparing two
      % residual norms as being close to each other
      sigma = 1e-4;
      
      % Get the previous pose
      ok = this.PreviousEstimate(:);
      
      % Iteration loop 
      while ~converged
        % Calculate new residual
        [fval, J] = lsqnonlin_target(this, ok, l);
        
        % Calculate change of state
        dely = lsqminnorm(J, fval);
        
        % Scaling factor for damped Newton-Raphson
        alpha = 1;
        
        % As long as we haven't halvened too often
        while alpha >= maxhalfs
          % Trial state update
          ok1_trial = ok - alpha * dely;

          % Calculate trial residual
          fval_trial = lsqnonlin_target(this, ok1_trial, l);

          % Check if lower residual, then we can stop this inner loop and
          % store the last value as the new value for the Newton iteration
          if norm(fval_trial, 2) ^ 2 <= norm(fval, 2) ^ 2 + sigma * alpha * ( norm(fval, 2) * norm(dely, 2) )
            % Push trial residual to be our actual residual
            fval = fval_trial;
            
            % Break loop
            break
          % No smaller residual, so halven the step width
          else
            % Halven step width
            alpha = alpha / 2;
          end

        end
        % END inner damped Newton-Raphson loop

        % Store last trial state as next iteration state
        ok1 = ok1_trial;
        
        % Check if we have converged
        [converged, ~] = has_converged(this, kiter, fval, dely);
        
        % Push state if not converged
        ok = ok1;
        
        % Advance step counter
        kiter = kiter + 1;
        
      end
      
      % Return argument
      o = ok;
      
    end
    
    
    function [tf, f] = has_converged(this, iter, r, dely)
      %% HAS_CONVERGED checks if the iteration has converged
      %
      %   HAS_CONVERGED(K, R, DELY) validates, at iteration K, the residual R,
      %   and the change in state DELY against the tolerances set on this
      %   simulator.
      %
      %   Inputs:
      %
      %   K             Current iteration loop. If set to 0, then the function
      %                 will reset itself i.e., reload all persistent properties
      %                 with initial values.
      %
      %   R             Kx1 vector of residual of the target function.
      %
      %   DELY          Nx1 vector of change in state which to inspect.
      %
      %   Outputs:
      %
      %   TF            True or false flag indicating if the iteration process
      %                 can be considered converged.
      %
      %   F             Flag which type of convergence was achieved. Values are
      %                  -1: Max number of iterations reached
      %                   1: Residual of the function is smaller than the
      %                   function and constraint tolerances. 
      %                   2: Change in state is smaller than the step size
      %                     tolerance.
      %                   3: Change in state is small compared to the actual
      %                     state.
      
      
      % Residual is very small
      if 0.5 * ( norm(r, 2) ^ 2 ) <= this.SolverOptions.FunctionTolerance ^ 2
        tf = true;
        f = 1;
      % Change in state is smaller than an acceptable tolerance
      elseif norm(dely, 2) <= this.SolverOptions.StepTolerance
        tf = true;
        f = 2;
      elseif iter >= this.SolverOptions.MaxIterations
        tf = true;
        f = -1;
      % No convergance yet attained
      else
        tf = false;
        f = 0;
      end
      
    end
    
  end
  
  
  
  %% PROPERTIES AND STATES
  methods ( Access = protected )
    
    function validatePropertiesImpl(this)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values
      
      
      % Determine spatial dimension of the anchors (frame and platform)
      % depending on motion pattern
      switch this.MotionPattern
        case '1T'
          nra = 1;
          nrb = 0;
        
        case '2T'
          nra = 2;
          nrb = 0;
        
        case '3T'
          nra = 3;
          nrb = 0;
        
        case '1R2T'
          nra = 2;
          nrb = 2;
        
        case '2R3T'
          nra = 3;
          nrb = 3;
        
%         case '3R3T'
        otherwise
          nra = 3;
          nrb = 3;
      end
      
      % Check count of frame anchors matches
      validateattributes(this.Frame, {'numeric'}, {'2d', 'nrows', nra, 'nonempty',  'finite', 'nonnan', 'nonsparse'}, mfilename, 'Frame');
      
      % Validate platform anchors (if there must be any platform anchors)
      if nrb > 0
        validateattributes(this.Platform, {'numeric'}, {'2d', 'nrows', nrb, 'ncols', size(this.Frame, 2), 'nonempty',  'finite', 'nonnan', 'nonsparse'}, mfilename, 'Platform');
      end
      
    end
    
    
%     function processTunedPropertiesImpl(this)
%       %% PROCESSTUNEDPROPERTIESIMPL
%       % Perform calculations if tunable properties change while system is
%       % running
%       
%       
%     end
    

    function flag = isInactivePropertyImpl(this, prop)
      %% ISINACTIVEPROPERTYIMPL
      % Return false if property is visible based on object configuration, for
      % the command line and System block dialog
      
      
      switch prop
        case 'Platform'
          % Platform anchors are visible if motion pattern allows for rotations
          flag = any(strcmp(this.MotionPattern, {'1T', '2T', '3T'}));
        
        otherwise
          flag = false;
        
      end
      
    end
        
    
%     function ds = getDiscreteStateImpl(this)
%       %% GETDISCRETESTATEIMPL
%       % Return structure of properties with DiscreteState attribute
%       
%       
%       ds = struct([]);
%       
%     end
    
  end
  
  
  
  %% INPUTS AND OUTPUTS
  methods ( Access = protected )

    function validateInputsImpl(this, l)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      % Ensure input pose is of right dimensions
      validateattributes(l, {'numeric'}, {'vector', 'numel', size(this.Frame, 2), 'nonempty', 'finite', 'nonsparse'}, mfilename, 'l');
      
    end
    
    
    function flag = isInputSizeLockedImpl(this, index)
      %% ISINPUTSIZELOCKEDIMPL
      % Return true if input size is not allowed to change while system is
      % running
      
      
      flag = true;
      
    end
    
    
    function num = getNumInputsImpl(this)
      %% GETNUMINPUTSIMPL
      % Define total number of inputs for system with optional inputs
      
      
      % P = STEP(L)
      num = 1;
      
      % if this.UseOptionalInput
      %   num = 2;
      % end
      
    end
    
    
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional outputs
      
      
      % P = STEP(L)
      num = 2;
      
      % if this.UseOptionalOutput
      %   num = 2;
      % end
      
    end
  
  end
  
  
  
  %% LOADING,SAVING, AND STATUS
  methods ( Access = protected )
    
%     function loadObjectImpl(this, s, wasLocked)
%       %% LOADOBJECTIMPL
%       % Set properties in object this to values in structure s
% 
%       % Set private and protected properties
%       % this.myproperty = s.myproperty; 
% 
%       % Set public properties and states
%       loadObjectImpl@matlab.System(this, s, wasLocked);
%       
%     end
    
    
%     function s = saveObjectImpl(this)
%       %% SAVEOBJECTIMPL
%       % Set properties in structure s to values in object this
% 
%       % Set public properties and states
%       s = saveObjectImpl@matlab.System(this);
% 
%       % Set private and protected properties
%       %s.myproperty = this.myproperty;
%       
%     end
    
    
%     function status = isDoneImpl(this)
%       %% ISDONEIMPL
%       % Return true if end of data has been reached
%       
%       
%       
%       status = false;
%     end
    
    
%     function s = infoImpl(this)
%       %% INFOIMPL
%       % Return structure of information about object this
%       
%       
%       s = struct([]);
%       
%     end
    
  end
  
  
  
  %% SYSTEM BLOCK ICON AND PORT LABELS
  methods ( Access = protected )
    
    function icon = getIconImpl(this)
      %% GETICONIMPL
      % Define icon for System block
      
      
      icon = mfilename('class'); % Use class name
      % icon = 'My System'; % Example: text icon
      % icon = {'My','System'}; % Example: multi-line text icon
      % icon = matlab.system.display.Icon('myicon.jpg'); % Example: image file icon
      
    end
    
    
    function name = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL
      % Return input port names for System block
      
      
      name = 'l';  
    end
    
    
    function [name1, name2] = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      % Return output port names for System block
      
      
      name1 = 'p';
      name2 = 'e';    
      
    end
    
    
    function [out1, out2] = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      out1 = 'double';
      out2 = 'double';

      % Example: inherit data type from first input port
      % out = propagatedInputDataType(this, 1);
      
    end
    
    
    function [out1, out2] = isOutputComplexImpl(this)
      %% ISOUTPUTCOMPLEXIMPL
      % Return true for each output port with complex data
      
      
      out1 = false;
      out2 = false;

      % Example: inherit complexity from first input port
      % out = propagatedInputComplexity(this, 1);
      
    end
    
    
    function [out1, out2] = isOutputFixedSizeImpl(this)
      %% ISOUTPUTFIXEDSIZEIMPL
      % Return true for each output port with fixed size
      
      
      out1 = true;
      out2 = true;

      % Example: inherit fixed-size status from first input port
      % out = propagatedInputFixedSize(this, 1);
      
    end
    
    
    function [sz, dt, cp] = getDiscreteStateSpecificationImpl(this, name)
      %% GETDISCRETESTATESPECIFICATIONIMPL
      % Return size, data type, and complexity of discrete-state specified in
      % name
      
      
      switch name
        case 'PreviousEstimate'
          sz = [1, this.NPose];
          dt = 'double';
          cp = false;
      end
      
    end
    
    
%     function sts = getSampleTimeImpl(this)
%       %% GETSAMPLETIMEIMPL
%       % Define sample time type and parameters
%       
%       
%       sts = this.createSampleTime('Type', 'Inherited');
% 
%       % Example: specify discrete periodic sample time
%       % sts = this.createSampleTime('Type', 'Discrete Periodic', ...
%       %   'SampleTime', 1);
%     end
    
  end
  
  
  %% SIMULINK NONDIRECT FEEDTHROUGH
  methods ( Access = protected )
    
%     function updateImpl(this, u)
%       %% UPDATEIMPL
%       % Update discrete states as a function of input u
%       
%       
%     end
    
    
%     function y = outputImpl(this, u)
%       %% OUTPUTIMPLOUTPUTIMPL
%       % Calculate output y as a function of discrete states and direct
%       % feedthrough inputs
%       
%       
%     end
    
    
%     function flag = isInputDirectFeedthroughImpl(this, u)
%       %% ISINPUTDIRECTFEEDTHROUGHIMPL
%       % Return true if input u is needed to calculate the output at the same
%       % time
%       
%       
%       flag = true;
%       
%     end
    
  end
  
  
  %% SIMULINK MODELING
  methods ( Access = protected )

    function flag = supportsMultipleInstanceImpl(this)
      %% SUPPORTSMULTIPLEINSTANCEIMPL
      % Return true if System block can be used inside a For Each subsystem,
      % which requires multiple object instances
      
      
      flag = true;
      
    end
    
    
%     function flag = allowModelReferenceDiscreteSampleTimeInheritanceImpl(this)
%       %% ALLOWMODELREFERENCEDISCRETESAMPLETIMEINHERITANCEIMPL
%       % Return true if sample time inheritance is allowed in Model blocks
%       
%       
%       flag = true;
%       
%     end
    
    
%     function names = getGlobalNamesImpl(this)
%       %% GETGLOBALNAMESIMPL
%       % Return names of global variables defined in Data Store Memory blocks
%       
%       
%       names = {};
%       
%     end
    
  end
  
  
  
  %% SYSTEM BLOCK SIGNAL PROPAGATION
  methods ( Access = protected )
    
    function [out1, out2, varargout] = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL
      % Return size for each output port
      
      
      switch this.MotionPattern
        case '1T'
          np = 1 + 0;
        
        case '2T'
          np = 2 + 0;
        
        case '3T'
          np = 3 + 0;
        
        case '1R2T'
          np = 2 + 4;
        
        case '2R3T'
          np = 3 + 9;
          
%         case '3R3T'
        otherwise
          np = 3 + 9;
          
      end
      
      out1 = [1, np];
      out2 = [1, np];
      
      nout = 1;
      
    end
    
  end
  
  
  
  %% SYSTEM BLOCK DIALOG
  methods ( Access = protected, Static )
    
    function header = getHeaderImpl()
      %% GETHEADERIMPL
      % Define header panel for System block dialog
      
      
      header = matlab.system.display.Header(mfilename('class'));
      
    end
    
    
    function group = getPropertyGroupsImpl()
      %% GETPROPERTYGROUPSIMPL
      % Define property section(s) for System block dialog
      
      
      group = matlab.system.display.Section(mfilename('class'));
      
    end
    
    
    function simMode = getSimulateUsingImpl()
      %% GETSIMULATEUSINGIMPL
      % Return only allowed simulation mode in System block dialog
      
      
      simMode = 'Interpreted execution';
%       simMode = 'Code generation';
      
    end
    
    
    function flag = showSimulateUsingImpl()
      %% SHOWSIMULATEUSINGIMPL
      % Return false if simulation mode hidden in System block dialog
      
      
      flag = false;
      
    end
    
    
%     function flag = showFiSettingsImpl()
%       %% SHOWFISETTINGSIMPL
%       % Return true if fixed-point tab appears in System block dialog
%       
%       
%       flag = true;
%       
%     end
    
  end
  
  
  
  %% OBSERVER METHODS
  methods ( Static, Access = protected )
    
  end
  
  
  
  %% INTERNAL OBSERVER METHODS
  methods ( Access = protected )
    
  end
  
end

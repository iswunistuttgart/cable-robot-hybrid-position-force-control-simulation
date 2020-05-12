classdef standard < matlab.System ...
    ... & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon
  %%  STANDARD Standard inverse kinematics for a 1R2T CDPR
  %
  %   L = STANDARD.STEP(P) calculates the cable lengths L from the pose values
  %   P.
  %
  %   [L, U] = STANDARD.STEP(P) also returns the cables' unit vectors.
  %
  %   Inputs:
  %
  %   P                 6x1 vector of pose values [X, Y, R11, R12, R21, R22] to
  %                     use for solving the inverse kinematics.
  %
  %   Outputs:
  %
  %
  %   U                 2xM vector of cable unit vectors
  
  
  
  %% PUBLIC, TUNABLE PROPERTIES
  properties
    
  end
  
  
  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % Position of winches in world frame [ m ]
    Winch = zeros(2, 4)
      
    % Initial pose [ m ]
    InitialPose = zeros(1, 2)
    
    % Cable length offset [ m ]
    CableOffset = zeros(1, 4)
    
  end
  
  
  %% PUBLIC, LOGICAl, NON-TUNABLE PROPERTIES
  properties(Nontunable, Logical)
    
    % Return cable unit vectors
    UnitVectors = false
    
    % Return cable deltas
    DeltaLength = false;
    
  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )

  end
  
  
  %% CONSTANT HIDDEN
  properties(Constant, Hidden)
    
  end
  
  
  %% PRE-COMPUTED CONSTANTS
  properties ( Access = private )
    
    % Initial cable lengths [ m ]
    InitialCableLength
    
    % Number of wires [ ]
    NCables
    
  end

  
  
  %% GENERAL METHODS
  methods
    
    % Constructor
    function this = standard(varargin)
      %% STANDARD
      
      
      % Support name-value pair arguments when constructing object
      setProperties(this, nargin, varargin{:})
      
    end
    
  end
  
  
  
  %% PROTECTED METHODS
  methods(Access = protected)
    
    function setupImpl(this)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
      % Count number of cables
      this.NCables = size(this.Winch, 2);
      
      % Calculate initial cable length needed for returning only the delta
      % length
      this.InitialCableLength = this.stepImpl(this.InitialPose) + this.CableOffset;
      
    end
    
    
    function [l, varargout] = stepImpl(this, p)
      %% STEPIMPL calculates the output from the input
      
      
      % Position, repeated N columns
      r = repmat([p(1); p(2)], 1, this.NCables);


      % Cable vectors
      L = this.Winch - r;

      % Lengths of cables
      l = sqrt( sum( ( L ).^2, 1));

      % Process variable list of arguments
      nout = 1;

      % Cable unit vectors
      if nargout > nout && this.UnitVectors
        varargout{nout} = bsxfun(@rdivide, L, l);
        nout = nout + 1;
      end

      % Length delta
      if nargout > nout && this.DeltaLength
        varargout{nout} = l - this.InitialCableLength;
        nout = nout + 1;
      end
        
    end
    
    
    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties
      
      
    end
    
  
    function validatePropertiesImpl(obj)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values
      
      validateattributes(obj.Winch, {'numeric'}, {'2d', 'nrows', 2, 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Winch');
      
      validateattributes(obj.InitialPose, {'numeric'}, {'row', 'numel', 2 + 0, 'nonnegative', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Initial pose');
      
      validateattributes(obj.CableOffset, {'numeric'}, {'row', 'numel', size(obj.Winch, 2), 'nonnegative', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Cable offset');
      
    end
    
  end
  
  
  
  %% Backup/restore functions
  methods ( Access = protected )
    
    function s = saveObjectImpl(this)
      %% SAVEOBJECTIMPL
      % Set properties in structure s to values in object this
      
      
      % Set public properties and states
      s = saveObjectImpl@matlab.System(this);

      % Set private and protected properties
      %s.myproperty = this.myproperty;
      
    end
    
    
    function loadObjectImpl(this, s, wasLocked)
      %% LOADOBJECTIMPL
      % Set properties in object this to values in structure s
      
      
      % Set private and protected properties
      % this.myproperty = s.myproperty; 

      % Set public properties and states
      loadObjectImpl@matlab.System(this, s, wasLocked);
      
    end
  
  end
  
  
  
  %% Simulink functions
  methods ( Access = protected )
    
    function ds = getDiscreteStateImpl(this)
      %% GETDISCRETESTATEIMPL
      % Return structure of properties with DiscreteState attribute
      
      
      ds = struct([]);
      
    end
    
    
    function validateInputsImpl(this, p)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      validateattributes(p, {'numeric'}, {'vector', 'numel', 2 + 0, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'p');
      
    end
    
    
    function flag = isInputSizeLockedImpl(this, index)
      %% ISINPUTSIZELOCKEDIMPL
      % Return true if input size is not allowed to change while system is
      % running
      
      
      flag = true;
      
    end
    
  
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional
      % outputs
      
      
      % Just cable length
      num = 1;
      
      % With unit vectors
      if this.UnitVectors
        num = num + 1;
      end
      
      % With length deltas
      if this.DeltaLength
        num = num + 1;
      end
      
    end
    
    
    function [out1, varargout] = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL for each output port
      
      
      % L = STEP(P)
      out1 = [1, size(this.Winch, 2)];
      
      nout = 1;
      
      % [L, U] = STEP(P)
      if nargout > nout && this.UnitVectors
        varargout{nout} = [2, size(this.Winch, 2)];
        nout = nout + 1;
      end
      
      % [L, U, DL] = STEP(P)
      if nargout > nout && this.DeltaLength
        varargout{nout} = [1, size(this.Winch, 2)];
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      % cable lengths
      out1 = 'double';
      
      nout = 1;
      
      if nargout > nout && this.UnitVectors
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
      if nargout > nout && this.DeltaLength
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
    end
    
    
    function icon = getIconImpl(this)
      %% GETICONIMPL
      
      
      % Define icon for System block
      icon = {'2T', 'Inverse Kinematics', 'Standard'};
      
    end
    
    
    function name = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL
      
      
      % Return input port names for System block
      name = 'p';
      
    end
    
    
    function [name1, varargout] = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      
      
      % Return output port names for System block
      name1 = 'l';
      
      nout = 1;
      
      if nargout > nout && this.UnitVectors
        varargout{nout} = 'u';
        nout = nout + 1;
      end
      
      if nargout > nout && this.DeltaLength
        varargout{nout} = 'dl';
        nout = nout + 1;
      end
      
    end
    
  end
  
  
  
  %% STATIC PROTECTED METHODS
  methods(Static, Access = protected)
    
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
    
    
    function flag = showSimulateUsingImpl()
      %% SHOWSIMULATEUSINGIMPL
      % Return false if simulation mode hidden in System block dialog
      
      
      flag = false;
      
    end
    
  end
  
end

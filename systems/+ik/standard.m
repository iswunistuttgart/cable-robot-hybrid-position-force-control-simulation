classdef standard < matlab.System ... %#codegen
    ... & matlab.system.mixin.FiniteSource ...
    & matlab.system.mixin.CustomIcon ...
    & matlab.system.mixin.Propagates ...
    ... & matlab.system.mixin.SampleTime ...
    ... & matlab.system.mixin.Nondirect
    %% STANDARD Standard inverse kinematics of a CDPR
    %
    %   L = STANDARD(P) calculates the cable lenghts L given the CDPR pose P.
    %
    %   [L, U] = STANDARD(P) also returns the cable unit vectors U.
    %
    %   [L, U, A] = STANDARD(P) also returns the pulley swivel/orientation
    %   angles A.
    %
    %   Inputs:
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
    %
    %   Outputs:
    %
    %   L               1xM vector of absolute cable lengths for the given pose
    %                   P.
    %
    %   U               NxM vector of cable unit vectors where N depends on the
    %                   CDPRs motion pattern.
    %
    %   A               1xM vector of swivel/orientation angles of each
    %                   pulley/cable giving the direction between the X-axis and
    %                   the cable leaving the winch to the platform.
  
  
  %% PUBLIC, NON-TUNABLE LOGICALS
  properties ( Nontunable, Logical )
    
    % Return unit vectors
    UnitVectors = false
    
    % Return Pulley Angles
    PulleyAngles = false
    
    % Return cable deltas
    DeltaLength = false;
    
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
    
    % Initial pose [ m ]
    InitialPose = zeros(1, 6)
    
    % Additional cable length offset
    CableOffset = zeros(1, 1);
    
  end
  
  
  %% CONSTANT HIDDEN PROPERTIES
  properties ( Constant, Hidden )
    
    % Supported motion patterns
    MotionPatternSet = matlab.system.StringSet({'1T', '2T', '3T', '1R2T', '2R3T', '3R3T'});
    
  end
  
  
  %% PROTECTED PROPERTIES
  properties ( Access = protected )
    
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
    
  end
  
  
  %% PRIVATE PROPERTIES
  properties ( Access = private )
    
    % Initial cable lengths [ m ]
    InitialCableLength
    
  end
  
  
  
  %% CONSTRUCTOR
  methods
    
    function this = standard(varargin)
      %% STANDARD
      
      
      % And set properties
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
    
    function setupImpl(this, varargin)
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
      
      % Calculate initial cable length needed for returning only the delta
      % length
      this.InitialCableLength = this.stepImpl(this.InitialPose) + this.CableOffset;
      
    end
    
    
    function [l, varargout] = stepImpl(this, p)
      %% STEPIMPL
      
      
      switch this.MotionPattern
        case '1T'
          r = p(1);
          
          % 1xN
          L = this.Frame - ( r );
          
        case '2T'
          r = [p(1); p(2)];
          
          % 1xN
          L = this.Frame - ( r );
          
        case '3T'
          r = [p(1); p(2); p(3)];
          
          % 1xN
          L = this.Frame - ( r );
          
        case '1R2T'
          r = [p(1); p(2)];
          R = p([3, 4; 5, 6]);
          
          % 1xN
          L = this.Frame - ( repmat(r, 1, this.NCables) + R * this.Platform );
          
        case '2R3T'
          r = [p(1); p(2); p(3)];
          R = p([4, 5, 6; 7, 8, 9; 10, 11, 12]);
          
          % 1xN
          L = this.Frame - ( repmat(r, 1, this.NCables) + R * this.Platform );
          
%         case '3R3T'
        otherwise
          r = [p(1); p(2); p(3)];
          R = p([4, 5, 6; 7, 8, 9; 10, 11, 12]);
          
          % 1xN
          L = this.Frame - ( repmat(r, 1, this.NCables) + R * this.Platform );
          
      end
      
      % Calculate cable lengths inside workspace and add cable offset to it
      l = sqrt( sum( ( L ).^2, 1)) + this.CableOffset;
      
      % Additional output arguments
      nout = 1;
      
      % Return unit vectors?
      if this.UnitVectors && nargout > nout
        % Calculate cable directions
        varargout{nout} = L ./ repmat(l, size(L, 1), 1);
        
        % Increase output argument counter
        nout = nout + 1;
        
      end
      
      % Return pulley orientation angles?
      if this.PulleyAngles && nargout > nout
        switch this.MotionPattern
          case '1T'
            % 1T cable robots cannot have any pulley orientation angle
            varargout{nout} = zeros(1, this.NCables);
            
          case '2T'
            % Angle between Y-axis and X-axis is the pulley orientation angle
            varargout{nout} = -atan2(L(2,:), L(1,:));
            
          case '3T'
            % Angle between Y-axis and X-axis is the pulley orientation angle
            varargout{nout} = -atan2(L(2,:), L(1,:));
            
          case '1R2T'
            % Angle between Y-axis and X-axis is the pulley orientation angle
            varargout{nout} = -atan2(L(2,:), L(1,:));
            
          case '2R3T'
            % Angle between Y-axis and X-axis is the pulley orientation angle
            varargout{nout} = -atan2(L(2,:), L(1,:));
            
%           case '3R3T'
          otherwise
            % Angle between Y-axis and X-axis is the pulley orientation angle
            varargout{nout} = -atan2(L(2,:), L(1,:));
            
        end
        
        % Increase output argument counter
        nout = nout + 1;
        
      end
      
      % Length delta?
      if nargout > nout && this.DeltaLength
        varargout{nout} = l - this.InitialCableLength;
        nout = nout + 1;
      end
      
    end
    
    
    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties
      
      
    end
    
    
    function releaseImpl(this)
      %% RELEASEIMPL
      % Release resources, such as file handles
      
      
    end
    
  end
  
  
  
  %% PROPERTIES AND STATES
  methods ( Access = protected )
    
    function validatePropertiesImpl(this)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values
      
      
      % Determine number of linear DoF from motion pattern
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
          
        case '3R3T'
          nra = 3;
          nrb = 3;
        
      end
      
      % Check count of frame anchors matches
      validateattributes(this.Frame, {'numeric'}, {'2d', 'nrows', nra, 'nonempty',  'finite', 'nonnan', 'nonsparse'}, mfilename, 'Frame');
      
      % Number of cables
      nc = size(this.Frame, 2);
      
      % Cable offset must not be empty
      if ~isscalar(this.CableOffset)
        validateattributes(this.CableOffset, {'numeric'}, {'vector', 'nonempty', 'ncols', nc}, mfilename, 'Cable Offset');
      end
      
      % Validate platform anchors (if there must be any platform anchors)
      if nrb > 0
        validateattributes(this.Platform, {'numeric'}, {'2d', 'nrows', nrb, 'ncols', nc, 'nonempty',  'finite', 'nonnan', 'nonsparse'}, mfilename, 'Platform');
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

    function validateInputsImpl(this, p)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      % Determine dimension of state vector depending on motion pattern as
      % `linear + rotational`
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
      
      % Ensure input pose is of right dimensions
      validateattributes(p, {'numeric'}, {'vector', 'numel', np, 'nonempty', 'finite', 'nonsparse'}, mfilename, 'p');
      
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
      
      
      % STEP(P)
      num = 1;
      
    end
    
    
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional outputs
      
      
      % L = STEP(P)
      num = 1;
      
      % [L, U] = STEP(P)
      if this.UnitVectors
        num = num + 1;
      end
      
      % [L, A] = STEP(P)
      % [L, U, A] = STEP(P)
      if this.PulleyAngles
        num = num + 1;
      end
      
      % With length deltas
      if this.DeltaLength
        num = num + 1;
      end
      
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
      
      
      name = 'p';
      
    end
    
    
    function [name1, varargout] = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      % Return output port names for System block
      
      
      % Default return cable length
      name1 = 'l';
      
      % Variable output argument counter
      nout = 1;
      
      % Name of unit vectors
      if this.UnitVectors
        varargout{nout} = 'u';
        nout = nout + 1;
      end
      
      % Name of pulley angles
      if this.PulleyAngles
        varargout{nout} = 'a';
        nout = nout + 1;
      end
      
      % Name of delta length
      if this.DeltaLength
        varargout{nout} = 'dl';
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      out1 = 'double';
      
      nout = 1;
      
      % Unit vectors are doubles
      if this.UnitVectors
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
      % Pulley angles are doubles
      if this.PulleyAngles
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
      % Delta lengths are doubles
      if this.DeltaLength
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = isOutputComplexImpl(this)
      %% ISOUTPUTCOMPLEXIMPL
      % Return true for each output port with complex data
      
      
      out1 = false;
      
      nout = 1;
      
      % Unit vectors are non-complex
      if this.UnitVectors
        varargout{nout} = false;
        nout = nout + 1;
      end
      
      % Pulley angles are non-complex
      if this.PulleyAngles
        varargout{nout} = false;
        nout = nout + 1;
      end
      
      % DeltaLengths are non-complex
      if this.DeltaLength
        varargout{nout} = false;
        nout = nout + 1;
      end
      
    end
    
    
    function [out, varargout] = isOutputFixedSizeImpl(this)
      %% ISOUTPUTFIXEDSIZEIMPL
      % Return true for each output port with fixed size
      
      
      out = true;
      
      nout = 1;
      
      % Unit vectors are fixed size
      if this.UnitVectors
        varargout{nout} = true;
        nout = nout + 1;
      end
      
      % Pulley angles are fixed size
      if this.PulleyAngles
        varargout{nout} = true;
        nout = nout + 1;
      end
      
      % Delta Lengths are fixed size
      if this.DeltaLength
        varargout{nout} = true;
        nout = nout + 1;
      end

    end
    
    
%     function [sz, dt, cp] = getDiscreteStateSpecificationImpl(this, name)
%       %% GETDISCRETESTATESPECIFICATIONIMPL
%       % Return size, data type, and complexity of discrete-state specified in
%       % name
%       
%       
%       sz = [1 1];
%       dt = 'double';
%       cp = false;
%       
%     end
    
    
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
    
    function [out, varargout] = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL
      % Return size for each output port
      
      
      out = [1, size(this.Frame, 2)];
      
      nout = 1;
      
      % Unit vectors are doubles
      if this.UnitVectors
        varargout{nout} = [size(this.Frame, 1), size(this.Frame, 2)];
        nout = nout + 1;
      end
      
      % Pulley angles are doubles
      if this.PulleyAngles
        varargout{nout} = [1, size(this.Frame, 2)];
        nout = nout + 1;
      end
      
      % [L, U, DL] = STEP(P)
      if this.DeltaLength
        varargout{nout} = [1, size(this.Frame, 2)];
        nout = nout + 1;
      end
      
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

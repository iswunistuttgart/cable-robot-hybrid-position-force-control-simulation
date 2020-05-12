classdef structurematrix < matlab.System ... %#codegen
    ... & matlab.system.mixin.FiniteSource ...
    & matlab.system.mixin.CustomIcon ...
    & matlab.system.mixin.Propagates ...
    ... & matlab.system.mixin.SampleTime ...
    ... & matlab.system.mixin.Nondirect
    %% STRUCTUREMATRIX Determine structure matrix
    %
    %   AT = STRUCTUREMATRIX(P, U) calculates the structure matrix AT for cable
    %   unit vectors U at pose P. The pose is mostly only needed for cable
    %   robots of motion pattern 1R2T, 2R3T, and 3R3T, where the platform
    %   rotation is needed to determine the torque wrench components of AT.
    %
    %   [AT, J] = STRUCTUREMATRIX(P, U) also returns the Jacobian J = -AT'.
    %
    %   [AT, J, N] = STRUCTUREMATRIX(P, U) also returns the nullspace N of
    %   structure matrix AT.
    %
    %   Inputs:
    %
    %   U               NLxM matrix of cable unit vectors.
    %
    %   P               1xNY vector of platform pose.
  
  
  %% PUBLIC, NON-TUNABLE LOGICALS
  properties ( Nontunable, Logical )
    
    % Return Jacobian
    Jacobian = false;
    
    % Return nullspace
    Nullspace = false;
    
  end
  
  
  %% PUBLIC, NON-TUNABLE INTEGERS
  properties ( Nontunable, PositiveInteger )
    
  end
  
  
  %% PUBLIC NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % CDPR motion pattern
    MotionPattern = '3R3T';
    
    % Anchors of cable on platform
    Platform = zeros(3, 1);
    
  end
  
  
  %% CONSTANT HIDDEN PROPERTIES
  properties ( Constant, Hidden )
    
    % Available motion patterns for IK
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
    
  end
  
  
  
  %% CONSTRUCTOR
  methods
    
    function this = structurematrix(varargin)
      %% STRUCTUREMATRIX
      
      
      setProperties(this, nargin, varargin{:})
      
      % Number of cables
      this.NCables = size(this.Platform, 2);
      
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
      
    end
    
  end
  
  
  
  %% SETTERS
  methods
    
  end
  
  
  
  %% ALGORITHM
  methods ( Access = protected )
    
    function setupImpl(this, p, u)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
      % Determine number of cables from number of columns of input U
      this.NCables = size(u, 2);
      
    end
    
    
    function [At, varargout] = stepImpl(this, p, u)
      %% STEPIMPL
      
      
      % Calculate structure matrix depending on motion pattern
      switch this.MotionPattern
        case '1T'
          % Force part of structure matrix
          At = u;
          
        case '2T'
          % Force part of structure matrix
          At = u;
          
        case '3T'
          % Force part of structure matrix
          At = u;
          
        case '1R2T'
          % Get rotation matrix from pose
          R = p([3, 4; 5, 6]);

          % Orient platform anchors into world frame
          bi = R * this.Platform;
          
          % Build structure matrix
          At = vertcat( ...
            u ... % Force part
            , bi(1,:) .* u(2,:) - bi(2,:) .* u(1,:) ... % Torque part
          );
          
        case '2R3T'
          % Get rotation matrix from pose
          R = p([4, 5, 6; 7, 8, 9; 10, 11, 12]);

          % Build structure matrix
          At = vertcat( ...
            u ... % Force part
            , cross(R * this.Platform, u) ... % Torque part
          );
          
%         case '3R3T'
        otherwise
          % Get rotation matrix from pose
          R = p([4, 5, 6; 7, 8, 9; 10, 11, 12]);

          % Build structure matrix
          At = vertcat( ...
            u ... % Force part
            , cross(R * this.Platform, u) ... % Torque part
          );
          
      end
      
      % Additional output arguments
      nout = 1;
      
      % Return Jacobian?
      if this.Jacobian && nargout > nout
        % Calculate Jacobian matrix
        varargout{nout} = -transpose(At);
        % Increase output argument counter
        nout = nout + 1;
      end
      
      % Return nullspace?
      if this.Nullspace && nargout > nout
        % Calculate Nullspace of structure matrix
        varargout{nout} = null(At);
        % Increase output argument counter
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
      
      
      % Determine spatial dimension of the anchors (frame and platform)
      % depending on the motion pattern
      switch this.MotionPattern
        case '1T'
          nrb = 0;
        
        case '2T'
          nrb = 0;
        
        case '3T'
          nrb = 0;
        
        case '1R2T'
          nrb = 2;
        
        case '2R3T'
          nrb = 3;
        
%         case '3R3T'
        otherwise
          nrb = 3;
          
      end
      
      % Validate platform anchors (if there must be any platform anchors)
      if nrb > 0
        validateattributes(this.Platform, {'numeric'}, {'2d', 'nrows', nrb, 'nonempty',  'finite', 'nonnan', 'nonsparse'}, mfilename, 'Platform');
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
          % Hide platform anchors if CDPR has no rotation
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

    function validateInputsImpl(this, p, u)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      % Determine dimension of state vector depending on motion pattern as
      % `linear + rotational`
      switch this.MotionPattern
        case '1T'
          npn = 1 + 0;
          nuc = 0;
          
        case '2T'
          npn = 2 + 0;
          nuc = 0;
          
        case '3T'
          npn = 3 + 0;
          nuc = 0;
          
        case '1R2T'
          npn = 2 + 4;
          nuc = size(this.Platform, 2);
          
        case '2R3T'
          npn = 3 + 9;
          nuc = size(this.Platform, 2);
          
%         case '3R3T'
        otherwise
          npn = 3 + 9;
          nuc = size(this.Platform, 2);
          
      end
      
      % Validate pose dimensions
      validateattributes(p, {'numeric'}, {'vector', 'numel', npn, 'nonempty', 'finite', 'nonsparse'}, mfilename, 'p');
      
      % Validate columns of U if there is rotation
      if nuc ~= 0
%         % Number of cables
%         nc = size(this.Platform, 2);

        % Validate unit vectors dimensions
        validateattributes(u, {'numeric'}, {'2d', 'ncols', nuc, 'nonempty', 'finite', 'nonsparse'}, mfilename, 'u');
      end
      
    end
    
    
    function flag = isInputSizeLockedImpl(~, ~)
      %% ISINPUTSIZELOCKEDIMPL
      % Return true if input size is not allowed to change while system is
      % running
      
      
      flag = true;
      
    end
    
    
    function num = getNumInputsImpl(~)
      %% GETNUMINPUTSIMPL
      % Define total number of inputs for system with optional inputs
      
      
      % STEP(P, U)
      num = 2;
      
    end
    
    
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional outputs
      
      
      % AT = STEP(P, U)
      num = 1;
      
      % [AT, J] = STEP(P, U)
      if this.Jacobian
        num = num + 1;
      end
      
      % [AT, J] = STEP(P, U)
      % [AT, J, N] = STEP(P, U)
      if this.Nullspace
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
    
    
    function [name1, name2] = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL
      % Return input port names for System block
      
      
      name1 = 'p';
      
      name2 = 'u';
      
    end
    
    
    function [name1, varargout] = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      % Return output port names for System block
      
      
      % Default return structure matrix
      name1 = 'At';
      
      % Variable output argument counter
      nout = 1;
      
      % Name of Jacobian
      if this.Jacobian
        varargout{nout} = 'J';
        nout = nout + 1;
      end
      
      % Name of Nullspace
      if this.Nullspace
        varargout{nout} = 'N';
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      % Default return structure matrix
      out1 = 'double';
      
      % Variable output argument counter
      nout = 1;
      
      % Jacobian is double
      if this.Jacobian
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
      % Nullspace is double
      if this.Nullspace
        varargout{nout} = 'double';
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = isOutputComplexImpl(this)
      %% ISOUTPUTCOMPLEXIMPL
      % Return true for each output port with complex data
      
      
      % Default return structure matrix
      out1 = false;
      
      % Variable output argument counter
      nout = 1;
      
      % Jacobian is non-complex
      if this.Jacobian
        varargout{nout} = false;
        nout = nout + 1;
      end
      
      % Nullspace is non-complex
      if this.Nullspace
        varargout{nout} = false;
        nout = nout + 1;
      end
      
    end
    
    
    function [out1, varargout] = isOutputFixedSizeImpl(this)
      %% ISOUTPUTFIXEDSIZEIMPL
      % Return true for each output port with fixed size
      
      
      % Default return structure matrix
      out1 = true;
      
      % Variable output argument counter
      nout = 1;
      
      % Jacobian is fixed size
      if this.Jacobian
        varargout{nout} = true;
        nout = nout + 1;
      end
      
      % Nullspace is fixed size
      if this.Nullspace
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
    
    function [out1, varargout] = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL
      % Return size for each output port
      
      
      
      % Determine dimension of state vector depending on motion pattern as
      % `linear + rotational`
      switch this.MotionPattern
        case '1T'
          nr = 1;
          
        case '2T'
          nr = 2;
          
        case '3T'
          nr = 3;
          
        case '1R2T'
          nr = 3;
          
        case '2R3T'
          nr = 6;
          
%         case '3R3T'
        otherwise
          nr = 6;
          
      end
      
      % Number of cables going in
      su = propagatedInputSize(this, 2);
      if ~isempty(su)
        nc = su(2);
      else
        nc = [];
      end
      
      % Structure matrix has as many columns as there are cable unit vectors
      % going in
%       out1 = [nr, nc];
      out1 = propagatedInputSize(this, 2);
      
      % Variable output argument counter
      nout = 1;
      
      % Jacobian is just the transpose of the structure matrix, so number of
      % rows and columns are changed
      if this.Jacobian
        varargout{nout} = [nc, nr];
        nout = nout + 1;
      end
      
      % Nullspace depends on the number of cables and DOF
      if this.Nullspace
        varargout{nout} = [nc, min(nc - nr, 0)];
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
  
end

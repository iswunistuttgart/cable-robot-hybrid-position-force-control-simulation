classdef wmean < matlab.System ... %#codegen
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
    %   Pset            1xN vector of absolute cable unit vector for the given pose
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
    %
  
  
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
    
    % Cable inital length
    Cable_InitialLength = zeros(1, 8);

  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )
    
  end
  
  
  %% PROTECTED NON-TUNABLE PROPERTIES
  properties ( Nontunable, Access = protected )
    
    
  end

  
  %% CONSTANT HIDDEN PROPERTIES
  properties ( Constant, Hidden )
    
    % Supported motion patterns
    MotionPatternSet = matlab.system.StringSet({'1T', '2T', '3T', '1R2T', '2R3T', '3R3T'});
    
  end
  
  
  %% PROTECTED PROPERTIES
  properties ( Access = protected )
    
    % Inverse kinematics object
    ik@ik.standard
    
    % Structure matrix object
    sm@sm.structurematrix
    
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
    
    function this = qfac(varargin)
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
          this.sm = sm.structurematrix('MotionPattern', this.MotionPattern,'Jacobian', true);
        
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
     
      
    end
    
    
    function [p, g, pm] = stepImpl(this, l, pset, vset)
      %% STEPIMPL
      % Implement algorithm. Calculate y as a function of input u and discrete
      % states.
      
      switch this.MotionPattern
        case '1T'

        
        case '2T'
          [~, uo] = this.ik(pset);
          [~, J] = this.sm(pset, uo);
          g = abs((-J*vset.')./sqrt(sum(vset.^2)));
          p = sum(this.Frame,2)./this.NCables - sum(((g.*(l.')).'.*uo),2)./sum(g);
          pm = (sum(this.Frame,2) - sum((l.*uo),2))./this.NCables;
          p = p.';
          pm = pm.';
        
        case '3T'

        
        case '1R2T'

        
        case '2R3T'

        
%         case '3R3T'
        otherwise

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
  
  
  
  %% OPTIMIZATION METHODS
  methods ( Access = protected )
    
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

    function validateInputsImpl(this, l, pset, vset)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      % Ensure input pose is of right dimensions
      validateattributes(l, {'numeric'}, {'vector', 'numel', size(this.Frame, 2), 'nonempty', 'finite', 'nonsparse'}, mfilename, 'l');
      validateattributes(pset, {'numeric'}, {'vector', 'numel', size(this.Frame, 1), 'nonempty', 'finite', 'nonsparse'}, mfilename, 'pset');
      validateattributes(vset, {'numeric'}, {'vector', 'numel', size(this.Frame, 1), 'nonempty', 'finite', 'nonsparse'}, mfilename, 'vset');

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
      num = 3;
      
      
      % if this.UseOptionalInput
      %   num = 2;
      % end
      
    end
    
    
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional outputs
      
      
      % P = STEP(L)
      num = 3;
      
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
    
    
    function [name1, name2, name3] = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL
      % Return input port names for System block
      
      
      name1 = 'l';
      name2 = 'pset';
      name3 = 'vset';
      
    end
    
    
    function [name1, name2, name3] = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      % Return output port names for System block
      
      
      name1 = 'p';
      name2 = 'g';
      name3 = 'pw';
      
    end
    
    
    function [out1, out2, out3] = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      out1 = 'double';
      out2 = 'double';
      out3 = 'double';
      
      % Example: inherit data type from first input port
      % out = propagatedInputDataType(this, 1);
      
    end
    
    
    function [out1, out2, out3] = isOutputComplexImpl(this)
      %% ISOUTPUTCOMPLEXIMPL
      % Return true for each output port with complex data
      
      
      out1 = false;
      out2 = false;
      out3 = false;

      % Example: inherit complexity from first input port
      % out = propagatedInputComplexity(this, 1);
      
    end
    
    
    function [out1, out2, out3] = isOutputFixedSizeImpl(this)
      %% ISOUTPUTFIXEDSIZEIMPL
      % Return true for each output port with fixed size
      
      
      out1 = true;
      out2 = true;
      out3 = true;

      % Example: inherit fixed-size status from first input port
      % out = propagatedInputFixedSize(this, 1);
      
    end
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
    
    function [out1, out2 out3, varargout] = getOutputSizeImpl(this)
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
      
      out2 = [4, 1];
      
      out3 = [1, np];
      
      nout = 3;
      
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

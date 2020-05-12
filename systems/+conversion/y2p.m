classdef y2p < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon
  %% Y2P converts CDPR state to pose
  %
  %   P = Y2P(Y) converts state Y to pose P.
  %
  %   Inputs:
  %
  %   Y             3x1 state vector of Y = [RX, RY].
  %
  %   Outputs:
  %
  %   P             6x1 vector of pose representing the state Y as
  %                 P = [RX, RY, R11, R12, R21, R22].



  %% PUBLIC, TUNABLE PROPERTIES
  properties
    
  end
  
  
  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )
   
  end
  
  
  %% PUBLIC, LOGICAl, NON-TUNABLE PROPERTIES
  properties ( Nontunable, Logical )
    
  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )

  end
  
  
  %% CONSTANT HIDDEN
  properties ( Constant, Hidden )
    
  end
  
  
  %% PRE-COMPUTED CONSTANTS
  properties ( Access = private )
    
  end

  
  
  %% GENERAL METHODS
  methods
    
    % Constructor
    function this = y2p(varargin)
      %% Y2P
      
      
      % Support name-value pair arguments when constructing object
      setProperties(this, nargin, varargin{:})
      
    end
    
  end
  
  
  
  %% PROTECTED METHODS
  methods(Access = protected)
    
    function setupImpl(this)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
    end
    
    
    function p = stepImpl(this, y)
      %% STEPIMPL calculates the output from the input
      
      
%       % Pre-calculate sine and cosine of orientation
%       sc = sin(y(3));
%       cc = cos(y(3));
      
      % Build pose
      p = [y(1), y(2)];
        
    end
    
    
    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties
      
      
    end
    
  
    function validatePropertiesImpl(this)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values
      
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
    
    function validateInputsImpl(this, y)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      validateattributes(y, {'numeric'}, {'row', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'y');
      
    end
    
    
    function flag = isInputSizeLockedImpl(this, index)
      %% ISINPUTSIZELOCKEDIMPL
      % Return true if input size is not allowed to change while system is
      % running
      
      
      flag = true;
      
    end
    
    
    function num = getNumInputsImpl(obj)
      %% GETNUMINPUTSIMPL
      % Define total number of inputs for system with optional inputs
      
      
      % P = Y2P(Y)
      num = 1;
      
    end
    
  
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional
      % outputs
      
      
      % P = Y2P(Y)
      num = 1;
      
    end
    
    
    function out = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL for each output port
      
      
      % P = Y2P(Y)
      out = [1, 2];
      
    end

    function out1 = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      % P = Y2P(Y)
      out1 = 'double';
      
    end
    
    
    function icon = getIconImpl(this)
      %% GETICONIMPL
      
      
      % Define icon for System block
      icon = {'2T', 'State to Pose'};
      
    end
    
    
    function name = getInputNamesImpl(obj)
      %% GETINPUTNAMESIMPL
      % Return input port names for System block
      
      
      name = 'y';
      
    end
    
    
    function name1 = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      
      
      % Return output port names for System block
      name1 = 'p';
      
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

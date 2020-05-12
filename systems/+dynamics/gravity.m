classdef gravity < matlab.System ...
    ... & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon
  %% GRAVITY Gravitational forces calculator
  %
  %   WG = GRAVITY() calculates forces of gravity on the platform.
  %
  %   Outputs:
  %
  %   WG            3x1 vector of wrench from gravitational forces.



  %% PUBLIC, TUNABLE PROPERTIES
  properties
    
  end
  
  
  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % Constant of gravitation [ kg m / s2 ]
    Gravity = 9.81
    
    % Platform weight [ kg ]
    Weight = 1
    
    % Axis of gravitation
    Direction = 'Y-'
  end
  
  
  %% PUBLIC, LOGICAl, NON-TUNABLE PROPERTIES
  properties ( Nontunable, Logical )
    
  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )

  end
  
  
  %% CONSTANT HIDDEN
  properties ( Constant, Hidden )
    
    % Possible directions of gravity
    DirectionSet = matlab.system.StringSet({'X+', 'X-', 'Y+', 'Y-'})
    
  end
  
  
  %% PRE-COMPUTED CONSTANTS
  properties ( Access = private )
    
    % Actual gravity vector
    GravityVector = [0, -1, 0];
    
  end

  
  
  %% GENERAL METHODS
  methods
    
    % Constructor
    function this = gravity(varargin)
      %% GRAVITY
      
      
      % Support name-value pair arguments when constructing object
      setProperties(this, nargin, varargin{:})
      
    end
    
  end
  
  
  
  %% PROTECTED METHODS
  methods(Access = protected)
    
    function setupImpl(this)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
      % Determine vector of gravity from direction selected
      switch this.Direction
        case 'X+'
          eg = [ 1.0;  0.0];
          
        case 'X-'
          eg = [-1.0;  0.0];
          
        case 'Y+'
          eg = [ 0.0;  1.0];
          
        case 'Y-'
          eg = [ 0.0; -1.0];
      end
      
      % Determine gravity vector
      this.GravityVector = this.Weight * this.Gravity .* eg;
      
    end
    
    
    function wg = stepImpl(this)
      %% STEPIMPL calculates the output from the input
      
      
      % Nothing to calculate, just output something
      wg = this.GravityVector;
        
    end
    
    
    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties
      
      
    end

  
    function validatePropertiesImpl(this)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values
      
      validateattributes(this.Weight, {'numeric'}, {'scalar', 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Weight');
      
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
    
    
    function validateInputsImpl(this)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
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
      
      
      % WG = STEP()
      num = 0;
      
    end
    
  
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional
      % outputs
      
      
      % WG = STEP()
      num = 1;
      
    end
    
    
    function out1 = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL for each output port
      
      
      % WG = STEP()
      out1 = [2, 1];
      
    end

    function out1 = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      % WG = STEP()
      out1 = 'double';
      
    end
    
    
    function icon = getIconImpl(this)
      %% GETICONIMPL
      
      
      % Define icon for System block
      icon = {'2T', 'Gravitational Wrench'};
      
    end
    
    
    function name1 = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      
      
      % Return output port names for System block
      name1 = 'wg';
      
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
    
    function c1 = isOutputFixedSizeImpl(this)
      c1 = true;
    end
    
   function c1 = isOutputComplexImpl(obj)
       c1 = false;
   end
    
  end
  
end

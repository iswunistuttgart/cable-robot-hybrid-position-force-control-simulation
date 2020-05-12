classdef platform < matlab.System ...
    ... & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon
  %%  PLATFORM Platform dynamics of a 1R2T CDPR mobile platform
  %
  %   DDY = STEPIMPL(WC, WG, WP) calculates the acceleration of the platform
  %   given all external wrenches.
  %
  %   Inputs:
  %
  %   WC            3x1 vector of wrench from cable forces.
  %
  %   WG            3x1 vector of wrench from gravitational forces.
  %
  %   WP            3x1 vector of wrench from process forces.
  %
  %   Outputs:
  %
  %   DDY           Acceleration of the platform's three degrees of freedom.



  %% PUBLIC, TUNABLE PROPERTIES
  properties
    
  end
  
  
  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % Platform weight [ kg ]
    Weight = 1
       
  end
  
  
  %% PUBLIC, LOGICAl, NON-TUNABLE PROPERTIES
  properties ( Nontunable, Logical )
    
  end
  
  
  %% DISCRETE STATES
  properties ( DiscreteState )

  end
  
  
  %% CONSTANT HIDDEN
  properties(Constant, Hidden)
    
  end
  
  
  %% PRE-COMPUTED CONSTANTS
  properties ( Access = private )
    
    % State full mass matrix
    MassMatrix = blockdiag(1, 1);

  end

  
  
  %% GENERAL METHODS
  methods
    
    % Constructor
    function this = platform(varargin)
      %% PLATFORM
      
      
      % Support name-value pair arguments when constructing object
      setProperties(this, nargin, varargin{:})
      
    end
    
  end
  
  
  
  %% PROTECTED METHODS
  methods(Access = protected)
    
    function setupImpl(this)
      %% SETUPIMPL
      % Perform one-time calculations, such as computing constants
      
      
      % Calculate global mass matrix
      this.MassMatrix = blockdiag(this.Weight .* eye(2, 2));
      
    end
    
    
    function ddy = stepImpl(this, wc, wg, wp)
      %% STEPIMPL calculates the output from the input
      
      
      % Just solve M * A = F for A.
      ddy = mldivide(this.MassMatrix, wc(:) + wg(:) + wp(:));
      % Make row vector
      ddy = (ddy(:)).';
        
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
    
    
    function validateInputsImpl(this, wc, wg, wp)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization
      
      
      validateattributes(wc, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'wc');
      
      validateattributes(wg, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'wg');
      
      validateattributes(wp, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'wp');
      
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
      
      
      % DDY = STEP(WC, WG, WP)
      num = 3;
      
    end
    
  
    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional outputs
      
      
      % DDY = STEP(WC, WG, WP)
      num = 1;
      
    end
    
    
    function out = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL for each output port
      
      
      out = [1, 2];
      
    end
    
    
    function out1 = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port
      
      
      % accelerations
      out1 = 'double';
      
    end
    
    
    function icon = getIconImpl(this)
      %% GETICONIMPL
      
      
      % Define icon for System block
      icon = {'2T', 'Platform Dynamics'};
      
    end
    
    
    function [name1, name2, name3] = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL
      
      
      % Return input port names for System block
      name1 = 'wc';
      
      name2 = 'wg';
      
      name3 = 'wp';
      
    end
    
    
    function name1 = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL
      
      
      % Return output port names for System block
      name1 = 'DDy';
      
    end
    
  end
  
  
  
  %% STATIC PROTECTED METHODS
  methods ( Static, Access = protected )
    
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

    function c1 = isOutputFixedSizeImpl(this)
      c1 = true;
    end
    
   function c1 = isOutputComplexImpl(this)
       c1 = false;
   end
    
%     function flag = showSimulateUsingImpl()
%       %% SHOWSIMULATEUSINGIMPL
%       % Return false if simulation mode hidden in System block dialog
%       
%       
%       flag = false;
%       
%     end
    
  end
  
end

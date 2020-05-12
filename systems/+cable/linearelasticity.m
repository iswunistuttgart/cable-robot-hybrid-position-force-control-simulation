classdef linearelasticity < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon
  %% LINEARELASTICITY Cable dynamics model: spring or parallel spring-damper
  %
  %   FC = STANDARD(L, Q) calculates the resulting cable forces F from cable
  %   length L and drive length Q for a linearly elastic cable model.
  %
  %   FC = STANDARD(L, Q, DL, DQ) calculates the resulting cable forces F from
  %   cable length L, cable length rate DL, drive length Q, and drive rate DQ
  %   for a linearly visco-elastic cable model.
  %
  %   Inputs:
  %
  %   L                 Mx1 vector of geoemtric cable lengths.
  %
  %   Q                 Mx1 vector of drive-wound cable lengths.
  %
  %   DL                Mx1 vector of geometric cable rates.
  %
  %   DQ                Mx1 vector of drive-wound cable rates.
  %
  %   Outputs:
  %
  %   FC                Mx1 vector of resulting cable forces.


  %% PUBLIC, TUNABLE PROPERTIES
  properties

  end


  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )

    % Normalized coefficient of elasticity [ kg / m ]
    Elasticity = 1e6*ones(1, 4);

    % Normalized coefficient of viscositty [ kg / s / m ]
    Viscosity = 1e3*ones(1, 4);

    % Length of reference for elasticity [ m ]
    Reference_Length = ones(1, 4);

    % Initial cable length [ m ]
    Initial_Length = zeros(1, 4);

    % Elasticity model to use
    Model = 'spring-damper'

  end


  %% PUBLIC, LOGICAl, NON-TUNABLE PROPERTIES
  properties ( Nontunable, Logical )

  end


  %% DISCRETE STATES
  properties ( DiscreteState )

  end


  %% CONSTANT HIDDEN
  properties(Constant, Hidden)

    % Available model types
    ModelSet = matlab.system.StringSet({'spring','spring-damper'})

  end


  %% PRE-COMPUTED CONSTANTS
  properties ( Access = private )

    % Flag if we use a damper, too
    HasDamper = false

    % Number of cables
    NCables = 0

  end



  %% GENERAL METHODS
  methods

    % Constructor
    function this = linearelasticity(varargin)
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


      % Flag if we use a damper material model
      this.HasDamper = strcmp(this.Model, 'spring-damper');

    end


    function fc = stepImpl(this, l, q, Dl, Dq)
      %% STEPIMPL calculates the output from the input


      % Just calculate F = E*Q + N*DQ taking into account slack cables

      % Local variables as this speeds up execution
      l0 = this.Initial_Length;
      lr = this.Reference_Length;

      % Calculate force of the spring elements
      elas = this.Elasticity .* lr ./ l;
      dell = l - ( l0 + q );
      fc = dell .* elas;

      % Get indices where the cable is slack
      idxSlack = dell < 0;

      % Add damper?
      if this.HasDamper
          % Cable velocities as row vector
          Dl = Dl(:).';
          
          % Get damper coefficients
          visco = this.Viscosity;

          % Calculate differences of velocities
          Ddel = Dl - Dq;

          % Calculate cable force based on a linear spring-damper
          % model
          fc = fc + Ddel.*visco;

          % Set forces to zero were cable is slack and being compressed
          idxSlack = idxSlack | ( Ddel < 0 );
      end

      % Set cable force to zero wherever the cable is slack i.e., either too
      % long compared to geometric difference and being compressed.
%       fc = fc .* (1/2 + 1/2*tanh(10000.*dell));
%       fc(idxSlack) = 0;

    end


    function resetImpl(this)
      %% RESETIMPL
      % Initialize / reset discrete-state properties


    end


    function validatePropertiesImpl(this)
      %% VALIDATEPROPERTIESIMPL
      % Validate related or interdependent property values

      validateattributes(this.Elasticity, {'numeric'}, {'vector', 'row', 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Elasticity');
  
      if this.HasDamper
        validateattributes(this.Viscosity, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Viscosity');
      end

      validateattributes(this.Reference_Length, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Reference Length');

      validateattributes(this.Initial_Length, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'nonnegative', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Initial Length');

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


    function validateInputsImpl(this, l, q, Dl, Dq)
      %% VALIDATEINPUTSIMPL
      % Validate inputs to the step method at initialization

      validateattributes(l, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'l');
      validateattributes(q, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'q');

      if this.HasDamper
%         validateattributes(Dl, {'numeric'}, {'vector', 'numel', numel(this.Viscosity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'Dl');
%         validateattributes(Dq, {'numeric'}, {'vector', 'numel', numel(this.Viscosity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'Dq');
      end

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


      % FC = STEP(L, Q);
      num = 2;

      % FC = STEP(L, Q, DL, DQ);
      if strcmp(obj.Model, 'spring-damper')
        num = num + 2;
      end

    end


    function num = getNumOutputsImpl(this)
      %% GETNUMOUTPUTSIMPL
      % Define total number of outputs for system with optional
      % outputs


      % FC = STEP(L, Q)
      % FC = STEP(L, Q, DL, DQ)
      num = 1;

    end


    function out = getOutputSizeImpl(this)
      %% GETOUTPUTSIZEIMPL for each output port


      % Pass through how many cables go in
      out = propagatedInputSize(this, 1);
%       out = [4,1];

    end


    function out1 = getOutputDataTypeImpl(this)
      %% GETOUTPUTDATATYPEIMPL
      % Return data type for each output port


      % accelerations
      out1 = 'double';

    end


    function out1 = isOutputComplexImpl(obj)
      %% ISOUTPUTCOMPLEXIMPL
      % Return true for each output port with complex data


      % FC = STEP(L, Q)
      % FC = STEP(L, Q, DL, DQ)
      out1 = false;

    end


    function out = isOutputFixedSizeImpl(obj)
      %% ISOUTPUTFIXEDSIZEIMPL
      % Return true for each output port with fixed size


      out = true;

    end


    function icon = getIconImpl(this)
      %% GETICONIMPL


      % Define icon for System block
      icon = {'2T', 'Cable Dynamics', 'Linear Elasticity'};

    end


    function varargout = getInputNamesImpl(this)
      %% GETINPUTNAMESIMPL


      % Return input port names for System block
      varargout{1} = 'l';

      varargout{2} = 'q';

      varargout{3} = 'Dl';

      varargout{4} = 'Dq';

    end


    function name1 = getOutputNamesImpl(this)
      %% GETOUTPUTNAMESIMPL


      % Return output port names for System block
      name1 = 'fc';

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

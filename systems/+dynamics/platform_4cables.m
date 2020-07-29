% -----------------
% Kathrin Rausch 
% 2020-06-11     
% -----------------

classdef platform_4cables < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon


% Class: PLATFORM_8T
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculation of Platform Dynamics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % This class calculates the linear acceleration of the platform from
    % Newton-Euler equation 
    % The Robot has two tranlational DOF and no rotational DOF
    %
    % Inputs: 
    %
    %   w_c   	2x1 wrench from cable forces and torques (w_c = AT * fc)
    %
    %   w_g   	2x1 wrench from gravitational forces (no torques)
    %
    %   w_p  	2x1 wrench from process forces and torques
    %
    % Outputs: 
    %
    %	ddy     2x1 linear acceleration in direction of 2 DOF
    %
   

    
  %% PUBLIC, NON-TUNABLE PROPERTIES
  properties ( Nontunable )
    
    % Platform weight [ kg ]
    Weight = 50;
    
  end
    

%% PRE-COMPUTED CONSTANTS
    properties ( Access = private )
    
        % mass matrix
        % the mass matrix is a [2x2] matrix (2 tranlational DOF, no rotation)
        MassMatrix = zeros(2,2);
       

end
    
%% Constructor   
    methods
        function this = platform_4cables(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end
    

%% ALGORITHM
    methods(Access = protected)
        
        % Setup
        function setupImpl(this, varargin) 

            % Calculate global mass matrix

            this.MassMatrix = this.Weight * eye(2);
        end

     
        
    	% Calculate Outputs from Inputs
        function ddy = stepImpl(this, w_c, w_g, w_p)

            
            % only tranlational movement -> Impulssatz
            % --> equation to solve: M*dyy = w_c + w_g + w_p

            ddy = mldivide(this.MassMatrix, w_c + w_g + w_p);
                 
        end
        
    end
    
    
%% VALIDATE PROPERTIES
% methods(Access = protected)
%     function validatePropertiesImpl(this)
%         % Validate related or interdependent property values
%       
%         validateattributes(this.Weight, {'numeric'}, {'scalar', 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Weight');
%       
%     end
% end

%% BACKUP / RESTORE FUNCTIONS
methods ( Access = protected )
    function s = saveObjectImpl(this)
        % Set properties in structure s to values in object this
      
      
        % Set public properties and states
      	s = saveObjectImpl@matlab.System(this);

      	% Set private and protected properties
      	%s.myproperty = this.myproperty;

    end
    
    
    function loadObjectImpl(this, s, wasLocked)
        % Set properties in object this to values in structure s
      
      
        % Set private and protected properties
        % this.myproperty = s.myproperty; 

        % Set public properties and states
        loadObjectImpl@matlab.System(this, s, wasLocked);
      
    end
  
end


%% SIMULINK FUNCTIONS
methods ( Access = protected )
    
	function ds = getDiscreteStateImpl(this)
        % Return structure of properties with DiscreteState attribute
        ds = struct([]);
      
    end
end


%% INPUTS AND OUTPUTS
    methods (Access = protected)
       
    % INPUTS-----------------------------------
        function validateInputsImpl(this,w_c, w_g, w_p)
            % Validate inputs to the step method at initialization
            validateattributes(w_c, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'w_c');
         	validateattributes(w_g, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'w_g');
            validateattributes(w_p, {'numeric'}, {'vector', 'numel', 2, 'nonnan', 'finite', 'nonsparse'}, mfilename, 'w_p');
        end
        
        
        function flag = isInputSizeLockedImpl(this, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
    
    
        function num = getNumInputsImpl(this)
            % Define total number of inputs for system with optional inputs
            num = 3;
        end
        
         
        function [name1, name2, name3] = getInputNamesImpl(this)
            % Return input port names for System block
            name1 = 'w_c';
            name2 = 'w_g';
            name3 = 'w_p';
        end
        
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            num = 1;
        end
        
        
      	function [name1] = getOutputNamesImpl(this)
        	% Return output port names for System block

          	name1 = 'ddy';
            
        end
        
        
        function [out1] = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
         	out1 = 'double';
 
        end
    
    
        function [out1] = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            out1 = false;
            
        end
 
    
    
        function [out1] = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            out1 = true;
            
        end
        
        
        function [out1] = getOutputSizeImpl(this)
            % Return size for each output port

            out1 = [2,1];

        end
        
    end
    

  
        
        

%% SYSTEM BLOCK ICON
    methods ( Access = protected)
        function icon = getIconImpl(this)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end
    end
    
    
%% SYSTEM BLOCK DIALOG
    methods ( Access = protected, Static )
    
        function header = getHeaderImpl()
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename('class'));
      
        end
    
        
        function group = getPropertyGroupsImpl()
            % Define property section(s) for System block dialog
            group = matlab.system.display.Section(mfilename('class'));
      
        end
    
    
        function simMode = getSimulateUsingImpl()
            % Return only allowed simulation mode in System block dialog
            simMode = 'Interpreted execution';
            % simMode = 'Code generation';
      
        end
    
    
        function flag = showSimulateUsingImpl()
            % Return false if simulation mode hidden in System block dialog
            flag = false;
      
        end
        
    end
    
    
    
end
    
    

    

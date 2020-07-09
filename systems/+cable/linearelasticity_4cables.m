% -----------------
% Kathrin Rausch 
% 2020-06-11     
% -----------------

classdef linearelasticity_4cables < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon


% Class: LINEARELASTICITY_8T 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Calculates the resulting cable forces for a CDPR with 4 cables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % This class calculates the resulting cable forces from actual cable 
    % length l, cable length rate dl (calculatet from platform dynamics
    % and inverse kinematics), drive length q and drive rate dq fro a
    % linearly visco-elastic cable model (spring-damper model)
    %
    % Inputs: 
    %
    %   l       4x1 vector of geoemtric cable lengths.
    %
    %   q   	4x1 vector of drive-wound cable lengths.
    %
    %   dl  	4x1 vector of geometric cable rates.
    %
    %   dq      4x1 vector of drive-wound cable rates.  
    %
    % Outputs: 
    %
    %   f_c  	4x1 vector of resulting cable forces.
    %
   

    
%% PUBLIC NON-TUNABLE PROPERTIES
    properties(Nontunable) 
        % Normalized coefficient of elasticity [ kg / m ]
        Elasticity = 172e3*ones(4,1);  % [N/m] Steifigkeit   % Wert von Valentin Schmidts Diss

        % Normalized coefficient of viscosity [ kg / s / m ]
        Viscosity = 0.03*172e3*ones(4,1);   % D�mpfung     % Annahme (Simulation zeigt Fehler bei keiner D�mpfung, vermutlich wegen Sprung auf Kraftregler)  

        % Length of reference for elasticity [ m ]
        Reference_Length = ones(4 ,1);

        % Initial cable length [ m ]
        Initial_Length = zeros(4,1);
    end
   



    
%% Constructor   
    methods
        function this = linearelasticity_4cables(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end
    

%% ALGORITHM
    methods(Access = protected)
        
        % Setup
        function setupImpl(this, varargin) 

        end

     
        
    	% Calculate Outputs from Inputs
        function f_c = stepImpl(this, l, q, dl, dq)
                  
            % Local variables as this speeds up execution
            l_0 = this.Initial_Length;
            l_r = this.Reference_Length;
            f_c_s = zeros(4,1);
            f_c_d = zeros(4,1);

            
            % spring elements
          	elas    = this.Elasticity;% .* l_r ./ l;
            delta_l = l - ( l_0 + q );
            
            
           	% damper elements
            visco   = this.Viscosity;
            delta_v = dl - dq;
           
            
            
         
            for i = 1:4
                if delta_l(i) > 0 
                    f_c_s(i) = delta_l(i)*elas(i);
                else
                    %f_c_s(i) = 0.1;
                    f_c_s(i) = delta_l(i)*elas(i);
                end
                
                if delta_v(i) > 0
                   	f_c_d(i) = delta_v(i)*visco(i);
                else
                    %f_c_d(i) = 0.1;
                    f_c_d(i) = delta_v(i)*visco(i);
                end
            end

            % sum forces
            f_c = f_c_s + f_c_d;
           
        end
    end


%% VALIDATE PROPERTIES
% methods(Access = protected)
%     function validatePropertiesImpl(this)
%       	% Validate related or interdependent property values
% 
%     	validateattributes(this.Elasticity, {'numeric'}, {'vector', 'row', 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Elasticity');
% 
%         validateattributes(this.Viscosity, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Viscosity');
%      
%         validateattributes(this.Reference_Length, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'positive', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Reference Length');
% 
%         validateattributes(this.Initial_Length, {'numeric'}, {'vector', 'row', 'numel', numel(this.Elasticity), 'nonnegative', 'nonnan', 'finite', 'nonsparse', 'nonempty'}, mfilename, 'Initial Length');
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
        function validateInputsImpl(this,l,q,dl,dq)
            % Validate inputs to the step method at initialization
            validateattributes(l, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'l');
            validateattributes(q, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'q');
            validateattributes(dl, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'dl');
            validateattributes(dq, {'numeric'}, {'vector', 'numel', numel(this.Elasticity), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'dq');

            
           end
        
        
        function flag = isInputSizeLockedImpl(this, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
    
    
        function num = getNumInputsImpl(this)
            % Define total number of inputs for system with optional inputs
            num = 4;
        end
        
         
        function [name1, name2, name3, name4] = getInputNamesImpl(this)
            % Return input port names for System block
            name1 = 'l';
            name2 = 'q';
            name3 = 'dl';
            name4 = 'dq';
        end
         
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            num = 1;
        end
        
        
      	function name1 = getOutputNamesImpl(this)
        	% Return output port names for System block

          	name1 = 'f_c';

            
        end
        
        
        function out1 = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
            out1 = 'double';

  
            
        end
    
    
        function out1 = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            out1 = false;
 
            
        end
 
    
    
        function out = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            out = true;
            
        end
        
        
        function [out1] = getOutputSizeImpl(this)
            % Return size for each output port

            out1 = [4,1];

        end
    end
    

  
        
        

%% SYSTEM BLOCK ICON
    methods( Access = protected)
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
    
    end
end
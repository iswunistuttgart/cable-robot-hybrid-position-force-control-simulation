% -----------------
% Kathrin Rausch 
% 2020-06-11     
% -----------------

classdef myclass < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon


% Class: MYCLASS 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Short description of <MYCLASS>
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % More description 
    %
    % Inputs: 
    %
    %   U       Description of Input U
    %
    %
    % Outputs: 
    %
    %   Y1    	Description of Output Y1
    %              
    %
    %   Y2      Description of Output Y2
    %
   

    
%% PUBLIC NON-TUNABLE PROPERTIES
    properties(Nontunable) 
        
    end
    

%% PROTECTED DEPENDENT PROPERTIES
  properties ( Access = protected )
       
  end
  

%% PRIVATE PROPERTIES
  properties(Access = private)
      
  end

    
%% Constructor   
    methods
        function this = myclass(varargin)
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
        function [Y1,Y2] = stepImpl(this, u)
            
                     
        end
    end
    
    
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



%% INPUTS AND OUTPUTS
    methods (Access = protected)
       
    % INPUTS-----------------------------------
        function validateInputsImpl(this,U)
            % Validate inputs to the step method at initialization
            
           end
        
        
        function flag = isInputSizeLockedImpl(this, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
    
    
        function num = getNumInputsImpl(this)
            % Define total number of inputs for system with optional inputs
            num = 1;
        end
        
         
        function name = getInputNamesImpl(this)
            % Return input port names for System block
            name = 'u';
        end
        
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            num = 2;
        end
        
        
      	function [name1, name2] = getOutputNamesImpl(this)
        	% Return output port names for System block

          	name1 = 'Y1';

         	name2 = 'Y2';
            
        end
        
        
        function [out1, out2] = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
            % cable length
            out1 = 'double';
            
            % unit length vector
            out2 = 'double';
  
            
        end
    
    
        function [out1, out2] = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            out1 = false;
            
            out2 = false;
 
            
        end
 
    
    
        function [out1, out2] = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            out1 = true;

            out2 = true;
            
        end
        
        
        function [out1, out2] = getOutputSizeImpl(this)
            % Return size for each output port

            out1 = [8,1];

            out2 = [8,3];
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
    
    
    
%% SIMULINK MODELING
    methods ( Access = protected )

        function flag = supportsMultipleInstanceImpl(this)
            % Return true if System block can be used inside a For Each subsystem,
            % which requires multiple object instances
            flag = true;
      
        end
    end
    
end
    
    

    

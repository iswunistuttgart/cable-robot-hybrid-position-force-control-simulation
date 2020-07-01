% -----------------
% Kathrin Rausch 
% 2020-06-13     
% -----------------

classdef structurematrix_4cables < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon


% Class: STRUCTUREMATRIX_8T
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % class to determine the structure matrix for a robot with 4 cables
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % The structure matrix and the inverse jacobian are determined from the cable 
    % unit vectors and the vector oft the actual platform pose
    %
    % Inputs: 
    %
    %   u  	4x2 matrix of cable unit vectors
    %
    %   p  	1x2 vector of platform pose
    %      	p must be formatted as 
    %     	[X, Y]
    %
    %
    % Outputs: 
    %
    %   AT      Structure Matrix
    %              
    %
    %   JIK     inverse Jacobian = -transpose(AT)
    %
   

%% PUBLIC, NON-TUNABLE LOGICALS
    properties ( Nontunable, Logical )
    
        % Return inverse Jacobian
        inv_Jacobian = false;
    
    end
    
%% PUBLIC NON-TUNABLE PROPERTIES
    properties(Nontunable) 
        
        % Platform anchors (bi)
        Platform = zeros(4,2);
        
    end
    

 
    
%% Constructor   
    methods
        function this = structurematrix_8t(varargin)
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
        function [AT, varargout] = stepImpl(this, p, u)
            
        	nout = 1;
            
            % build structure matrix
            % AT = [u1 ... u8;
            %       R*b1 x u1 ... R*b8 x u8]
            % no rotation, 2 tranlational DOF, 4 cables 
            %       -> AT [2x4] (no torque part, only forces)
            
            
            AT = transpose(u);
            
            
            % Return inverse Jacobian?
            if this.inv_Jacobian && nargout > nout
                varargout{nout} = -transpose(AT);
                nout = nout +1;
            end
        
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
        function validateInputsImpl(this,p)
            % Validate inputs to the step method at initialization        
        	validateattributes(p, {'numeric'}, {'vector', 'numel', 2, 'nonempty', 'finite', 'nonsparse'}, mfilename, 'p');
           end
        
        
        function flag = isInputSizeLockedImpl(this, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
    
    
        function num = getNumInputsImpl(this)
            % Define total number of inputs for system with optional inputs
            num = 2;
        end
        
         
        function [name1, name2] = getInputNamesImpl(this)
            % Return input port names for System block
            name1 = 'p';
            name2 = 'u';
        end
        
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            
            % [AT]
            num = 1;
            
            % [AT, J]
            if this.inv_Jacobian
                num = num +1 ;
            end
        end
        
        
      	function [name1, varargout] = getOutputNamesImpl(this)
        	% Return output port names for System block

          	name1 = 'AT';
            nout = 1;
            
          	if this.inv_Jacobian
                varargout{nout} = 'J_inv';
            	nout = nout +1 ;
            end           
         	
            
        end
        
        
        function [out1, varargout] = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
        	out1 = 'double';
            nout = 1;
            
          	if this.inv_Jacobian
                varargout{nout} = 'double';
            	nout = nout +1 ;
            end    
  
            
        end
    
    
        function [out1, varargout] = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            out1 = false;
            
            nout = 1;
            
          	if this.inv_Jacobian
                varargout{nout} = false;
            	nout = nout +1 ;
            end    
 
            
        end
 
    
    
        function [out1, varargout] = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            out1 = true;

            nout = 1;
            
          	if this.inv_Jacobian
                varargout{nout} = true;
            	nout = nout +1 ;
            end    
            
        end
        
        
        function [out1, varargout] = getOutputSizeImpl(this)
            % Return size for each output port

            out1 = [2,4];
            nout = 1;
            
          	if this.inv_Jacobian
                varargout{nout} = [4,2];
            	nout = nout +1 ;
            end    

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
    
    

    



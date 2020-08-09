% -----------------
% Kathrin Rausch 
% 2020-06-11     
% -----------------

classdef y2p < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon


% Class: Y2P 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Class to call transform position and angle to pose
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % More description 
    %
    % Inputs: 
    %
    %   y           [3x1] position (x,y) and angle (z) of platform
    %
    % Outputs: 
    %
    %   p           [1x6] Pose of the Robot [rx, ry, R11, R12, R21, R22]
    %
   

    

    
%% Constructor   
    methods
        function this = y2p(varargin)
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
        function [p] = stepImpl(this,y)
            p(1:2) = y(1:2);
            sc = sin(y(3));
            cc = cos(y(3));

            p (3:6) = [cc, -sc, sc, cc] ;
  
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
       
        
        function flag = isInputSizeLockedImpl(this, index)
            % Return true if input size is not allowed to change while
            % system is running
            flag = true;
        end
    
    
        function num = getNumInputsImpl(this)
            % Define total number of inputs for system with optional inputs
            num = 1;
        end
        
         
        function [name1, name2, name3, name4] = getInputNamesImpl(this)
            % Return input port names for System block
            name1 = 'y';
        end
        
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            num = 1;
        end
        
        
        function name1 = getOutputNamesImpl(this)
        	% Return output port names for System block

          	name1 = 'p_ist';

            
        end
        
        
        function out1 = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
            % cable length
            out1 = 'double';
  
            
        end
    
    
        function out1 = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            out1 = false;

 
            
        end
 
    
    
        function out1 = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            out1 = true;
            
        end
        
        
        function out1 = getOutputSizeImpl(this)
            % Return size for each output port

            out1 = [1,6];
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
    
    

    

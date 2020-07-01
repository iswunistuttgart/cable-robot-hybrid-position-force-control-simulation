% -----------------
% Kathrin Rausch 
% 2020-06-11     
% -----------------

classdef ik_standard_4cables < matlab.System ...
    & matlab.system.mixin.Propagates ...
    & matlab.system.mixin.CustomIcon

	% Class: IK_STANDARD 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Standard inverse kinematics for a cable-driven parallel robot (CDPR)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %
    % Calculates the cable lenths L for a given CDPR pose P
    % The CDPR in this case has 4 cables
    %
    % Inputs: 
    %
    %   P       N... number of DOF 
    %           Defines the pose (position T and orientation R) of the platform
    %           The CDPR has 2 tranlational DOF
    %
    %           P must be formatted as 
    %           [X, Y]
    %
    %
    % Outputs: 
    %
    %   l       [Mx1]   M... number of cables -> here: [4x1]
    %           vector of absolute cable lengths for the given pose P
    %              
    %
    %   u       [M x 3]   -> here: [4x3]
    %           vector of cable unit vectors pointing away from the
    %           platform
    %
    %
	%   delta_l [Mx1]   M... number of cables -> here: [4x1]
    %           vector of delta cable lengths for the given pose P
    


%% PUBLIC, NON-TUNABLE LOGICALS
    properties ( Nontunable, Logical )
    
        % Return unit vectors
        UnitVectors = false
    
        % Return cable deltas
        DeltaLength = false;
    
    end
    
    %% PUBLIC NON-TUNABLE PROPERTIES
    properties(Nontunable) 
        
        % Winch/pulley locations (ai)
        FrameAnchors = [ ...
            [-4.70,  4.30] ...
          ; [ 3.70,  3.20] ...
          ; [ 3.50, -5.20] ...
          ; [-4.50, -5.00] ...
        ];

        % Platform anchors (bi)
        Platform = zeros(4,2);
    
        % Initial pose [Nx1]
        InitialPose = zeros(2,1);
    
        % Additional cable length offset [8x1]
        CableOffset = zeros(4,1);
        
    end
    


%% PRIVATE PROPERTIES
  properties(Access = private)

   	% Initial cable lengths [m]
   	InitialCableLength = zeros(1,4);
      
  end

    
%% Constructor   
    methods
        function this = ik_standard_4cables(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(this,nargin,varargin{:})
        end
    end
    

%% ALGORITHM
    methods(Access = protected)
        
        % Setup CDPR
        function setupImpl(this, varargin) 
            
            % calculate initial cable lenth for returning delta l
            this.InitialCableLength = this.stepImpl(this.InitialPose);
        end

     
        
    	% calculate absolute cable lenghts from given pose + offset
        function [l, varargout] = stepImpl(this, p)
            % get pose from Input p
        	r = [p(1); p(2)];
            
            L = zeros(4, 2);
            l = zeros(4,1);
            
          	% calculate cable lengths inside workspace and add cable offset
            for i=1:4
                a = transpose(this.FrameAnchors(i,1:2));
                
               
                nout = 1;

                % calculate cable lengths in K0 with li = ai - r - Rbi 
                % bi = 0 --> li = ai-r
                L(i, 1:2) = transpose(a - r);
                                         
                % calculate output 1:
                % absolute cable lengths
                l(i) = sqrt(L(i,1)^2 + L(i,2)^2) + this.CableOffset(i);
                
                
                
                % Return unit vectors?
                if this.UnitVectors && nargout > nout
                    % calculate output 2:
                    % cable unit vectors
                    varargout{nout} = L ./ l;
                    
                    nout = nout + 1;
                end
           
                
                
                % Return deltas?
                if nargout > nout && this.DeltaLength
                    % calculate output 3:
                    % delta cable length
                    varargout{nout} = l- this.InitialCableLength;
                    nout = nout +1;
                end

            end

            
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
            num = 1;
        end
        
         
        function name = getInputNamesImpl(this)
            % Return input port names for System block
            name = 'p';
        end
        
        
    % OUTPUTS-----------------------------------
        function num = getNumOutputsImpl(this)
            % Define total number of outputs for system with optional outputs
            
            % [l]
            num = 1;
            
            % [l, u]
            if this.UnitVectors
                num = num + 1;
            end
            
            % [l, u, delta]
            if this.DeltaLength
                num = num +1;
            end
        end
        
        
     
        
        function [out1, varargout] = getOutputDataTypeImpl(this)
            % Return data type for each output port
        
            % cable length
            out1 = 'double';
            
            nout = 1;
            
            
         	% unit length vector
            if this.UnitVectors
                varargout{nout} = 'double';
                nout = nout +1;
            end

         	% delta length
           if this.DeltaLength
               varargout{nout} = 'double';
               nout = nout +1;
           end 
            
        end
    
    
        function [out1, varargout] = isOutputComplexImpl(this)
            % Return true for each output port with complex data
        
            % cable length
            out1 = false;
            
            nout = 1;
            
         	% unit length vector
            if this.UnitVectors
                varargout{nout} = false;
                nout = nout +1;
            end

         	% delta length
           if this.DeltaLength
               varargout{nout} = false;
               nout = nout +1;
           end  
            
        end
 
    
    
        function [out1, varargout] = isOutputFixedSizeImpl(this)
            % Return true for each output port with fixed size
        
            % cable length
            out1 = true;
            
            nout = 1;
            
         	% unit length vector
            if this.UnitVectors
                varargout{nout} = true;
                nout = nout +1;
            end

         	% delta length
           if this.DeltaLength
               varargout{nout} =true;
               nout = nout +1;
           end   
            
        end
        
        
        function [out1, varargout] = getOutputSizeImpl(this)
            % Return size for each output port
            
            % length [Mx1]
            out1 = [4,1];
            nout = 1;
            
          	% unit vectors [Mx3]
            if this.UnitVectors
                varargout{nout} = [4,2];
                nout = nout +1;
            end

            % delta length [Mx1]
            if this.DeltaLength
                varargout{nout} = [4,2];
                nout = nout +1;
            end  
        end
        
        
        function [name1, varargout] = getOutputNamesImpl(this)
        	% Return output port names for System block

         	% cable length
          	name1 = 'l';
            
            nout = 1;

         	% unit length vector
            if this.UnitVectors
                varargout{nout} = 'u';
                nout = nout +1;
            end

         	% delta length
           if this.DeltaLength
               varargout{nout} = 'delta_l';
               nout = nout +1;
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
    
    

    

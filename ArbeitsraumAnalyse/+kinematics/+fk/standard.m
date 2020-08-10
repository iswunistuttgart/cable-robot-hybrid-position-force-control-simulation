function [Pose, varargout] = standard(CableLength, PulleyPosition, CableAttachment, SolverOptions)%#codegen
% STANDARD estimates the robot pose with standard pulley kinematics
% 
%   POSE = STANDARD(CABLELENGTH, PULLEYPOSITION, CABLEATTACHMENT) estimates the
%   pose given the cable lengths for the robot defined defined by the given
%   pulley positions and cable attachment points.
%   
%   Inputs:
%   
%   CABLELENGTH:        Vector of 1xM cable lengths as given by measurement of
%       the inverse kinematics.
% 
%   PULLEYPOSITION:     Matrix of pulley positions w.r.t. the world frame. Each
%       pulley has its own column and the rows are the x, y, and z-value,
%       respectively i.e., PULLEYPOSITIONS must be a matrix of 3xM values. The
%       number of pulleyvs i.e., N, must match the number of cable attachment
%       points in CABLEATTACHMENT (i.e., its column count) and the order must
%       match the real linkage of pulley to cable attachment on the platform
% 
%   CABLEATTACHMENT:    Matrix of cable attachment points w.r.t. the platform
%       coordinate system. Each attachment point has its own column and the rows
%       are the x, y, and z-value, respectively, i.e., CABLEATTACHMENT must be a
%       matrix of 3xM values. The number of cables i.e., N, must match the
%       number of pulleys in PULLEYPOSITION (i.e., its column count) and the
%       order must match the real linkage of cable attachment on the platform to
%       pulley.
%
%   SOLVEROPTIONS:      A struct of optimization options to set for the
%       lsqnonlin solver. All values may be overwriten and this function makes
%       use of the following pre-overwriten options
%   
%           Algorithm:  'levenberg-marquardt'
%           Display:    'off'
%           TolX:       1e-12
% 
%   Outputs:
% 
%   POSE:               Estimated pose given as 1x12 vector with the
%   interpretation of
%       
%       pose = [x_e, y_e, z_e, R11_e, R12_e, R13_e, R21_e, R22_e, R23_e, R31_e,
%       R32_e, R33_e]



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-10-08
% Changelog:
%   2016-10-08
%       * Move into package '+fk'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%       * Add assertion through `validateattributes`
%   2016-09-19
%       * Rename to `standard`
%   2016-03-30
%       * Code cleanup
%   2015-08-05
%       * Initial release



%% Argument processing
% Three or four input arguments
narginchk(3, 4);
% Zero to two output arguments
nargoutchk(0, 2);

% Default solver options
if nargin < 4 || isempty(SolverOptions)
    SolverOptions = struct();
end

% Assertion of arguments
validateattributes(CableLength, {'numeric'}, {'nonempty', 'vector', 'numel', 12}, mfilename, 'CableLength', 1);
validateattributes(PulleyPosition, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'PulleyPosition', 2);
validateattributes(CableAttachment, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'CableAttachment', 3);
validateattributes(SolverOptions, {'struct'}, {'nonempty'}, mfilename, 'SolverOptions', 4);



%% Initialize variables
% Get the provided cable length
vCableLength = asrow(CableLength);
% Get the provided pulley positions
aPulleyPosition = PulleyPosition;
% Get the provided cable attachment points on the platform
aCableAttachment = CableAttachment;
% Final guessed pose x_guess = [x_g, y_g, z_g, a_g, b_g, c_g]
vPoseEstimate = zeros(1, 6);
% Additional solver options
stSolverOptionsGiven = SolverOptions;

% Estimate the initial pose
vInitialStateForOptimization = fk.pose_estimate(vCableLength, aPulleyPosition, aCableAttachment);



%% Perform optimization of estimation
% Initialize solver options
opSolverOptions = optimoptions('lsqnonlin');
opSolverOptions.Algorithm = 'levenberg-marquardt';
opSolverOptions.Display = 'off';
opSolverOptions.TolFun = 1e-8;
opSolverOptions.TolX = 1e-12;
% opSolverOptions.InitDamping = 1e-10;
% opSolverOptions.ScaleProblem = 'Jacobian';
% opSolverOptions.FinDiffType = 'central';

% Given any user-defined solver options? Process them now
if numel(stSolverOptionsGiven)
    % Get the fields of the struct provided
    ceFields = fieldnames(stSolverOptionsGiven);
    % And assign each given value to the solver options
    for iField = 1:numel(ceFields)
        opSolverOptions.(ceFields{iField}) = stSolverOptionsGiven.(ceFields{iField});
    end
end

% Optimization target function
inOptimizationTargetVectorFunction = @(vEstimatedPose) standard_TargetFunction(vEstimatedPose, vCableLength, aPulleyPosition, aCableAttachment);

% And now finally run the optimization
[xFinal, resnorm, residual, exitflag, output, lambda, jacobian] = lsqnonlin(inOptimizationTargetVectorFunction, ...
    vInitialStateForOptimization, ... % Initial state
    [], [], ... % lower and upper boundaries are not set
    opSolverOptions ... % Custom solver options
);



%% Post processing of the estimated pose
% Extract the position
vPosition = xFinal(1:3);
% Extract the quaternion rotation and ...
% vRotation = xFinal(4:7);
% Extract the yaw-pitch-roll rotation angles and ...
vRotation = xFinal(4:6)';
% ... transform it into a rotation matrix
aRotation = eul2rotm(fliplr(asrow(vRotation(1:3)./180*pi)), 'ZYX');
aRotation(abs(aRotation) < 2*eps) = 0;

% Build the final estimated pose
vPoseEstimate = [reshape(vPosition, 1, 3), rotm2row(aRotation)];



%% Assign output quantities
% First and only required output is the estimated pose
Pose = vPoseEstimate;

% Second output, first optional may be the output struct from the optimization
if nargout > 1
    stBenchmark = struct();
    stBenchmark.output = output;
    stBenchmark.resnorm = resnorm;
    stBenchmark.residual = residual;
    stBenchmark.exitflag = exitflag;
    stBenchmark.lambda = lambda;
    stBenchmark.jacobian = jacobian;
    
    varargout{1} = orderfields(stBenchmark);
end



end


function [VectorValuedFunction, Jacobian] = standard_TargetFunction(EstimatedPose, TargetCableLength, PulleyPositions, CableAttachments)

%% Preparing variables
% Number of cables
nNumberOfCables = size(PulleyPositions, 2);
% Parse input variables
vEstimatedPose = reshape(EstimatedPose, 1, 6);
aPulleyPositions = PulleyPositions;
aCableAttachments = CableAttachments;
aTargetCableLength = TargetCableLength;
% Extract the position
vPosition = vEstimatedPose(1:3);
% And rotation from the estimated pose
% vRotation = vEstimatedPose(4:7).';
vRotation = vEstimatedPose(4:6);
% Transform the rotation given in quaternions to a DCM (direct cosine
% matrix)
aRotation = eul2rotm(fliplr(asrow(vRotation(1:3)./180*pi)), 'ZYX');
aRotation(abs(aRotation) < 2*eps) = 0;
% Create the needed pose for the inverse kinematics algorithm composed of
% [x, y, z, R11, R12, R13, R21, R22, R23, R31, R32, R33]
vEstimatedPose = [reshape(vPosition, 1, 3), rotm2row(aRotation)];
% Array holding the Jacobian
aJacobian = zeros(nNumberOfCables, 6);



%% Calculate the cable length for the current pose estimate
% Calculate the cable lengths for the estimated pose using the standard inverse
% kinematics algorithm
vLengths = algoInverseKinematics_Standard(vEstimatedPose, aPulleyPositions, aCableAttachments);



%% And build the target optimization vector
% Get the vector difference of all cable lengths ...
vEvaluatedFunction = vLengths(:).^2 - aTargetCableLength(:).^2;

% Also calculate the Jacobian?
if nargout > 1
    % Code taken from WireCenter, therefore not super beautiful and not
    % following code conventions either, but for now it must work
    t1 = cos(vRotation(1));
    t2 = cos(vRotation(2));
    t3 = sin(vRotation(1));
    t4 = cos(vRotation(3));
    t5 = sin(vRotation(2));
    t6 = sin(vRotation(3));
    t7 = t1*t6;
    t8 = t3*t4;
    t9 = t7*t5-t8;
    t10 = t1*t4;
    t11 = t3*t6;
    t12 = t10*t5+t11;
    for iCable = 1:nNumberOfCables
        t13 = t2*aCableAttachments(1,iCable);
        t14 = t13*t1;
        t15 = t9*aCableAttachments(2,iCable);
        t16 = t12*aCableAttachments(3,iCable);
        t17 = vPosition(1)-aPulleyPositions(1,iCable)+t15+t16+t14;
        t10 = t11*t5+t10;
        t7 = t8*t5-t7;
        t8 = t13*t3;
        t11 = t10*aCableAttachments(2,iCable)+t7*aCableAttachments(3,iCable)+t8-aPulleyPositions(2,iCable)+vPosition(2);
        t18 = t4*aCableAttachments(3,iCable);
        t19 = t6*aCableAttachments(2,iCable);
        t20 = t19+t18;
        t21 = t5*aCableAttachments(1,iCable);
        t22 = t2*t20-t21-aPulleyPositions(3,iCable)+vPosition(3);
        t18 = -t2*(t19+t18)+t21;
        t19 = 0.2e1;
        aJacobian(iCable,:) = [t19*t17, ...
                t19*t11, ...
                t19*t22, ...
                -t19*(t17*(t10*aCableAttachments(2,iCable)+t7*aCableAttachments(3,iCable)+t8)-t11*(t15+t16+t14)), ...
                -t19*(t17*t1*t18+t11*t3*t18+t22*(t20*t5+t13)), ...
                t19*(t11*(-t10*aCableAttachments(3,iCable)+t7*aCableAttachments(2,iCable))+t17*(t12*aCableAttachments(2,iCable)-t9*aCableAttachments(3,iCable))+t22*t2*(t4*aCableAttachments(2,iCable)-t6*aCableAttachments(3,iCable)))];
    end
end



%% Assign output quantities
% ... which is our return value
VectorValuedFunction = vEvaluatedFunction;

% Assign the output Jacobian if requested
if nargout > 1
    Jacobian = aJacobian;
end

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

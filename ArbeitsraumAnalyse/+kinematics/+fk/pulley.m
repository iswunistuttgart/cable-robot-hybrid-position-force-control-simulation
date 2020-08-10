function [Pose, varargout] = pulley(CableLength, PulleyPosition, CableAttachment, PulleyRadius, PulleyOrientation, SolverOptions)%#codegen
% PULLEY estimates the robot pose with advanced pulley kinematics
% 
%   POSE = PULLEY(CABLELENGTH, PULLEYPOSITION, CABLEATTACHMENT) estimates the
%   pose given the cable lengths for the robot defined defined by the given
%   pulley positions and cable attachment points.
%
%   [POSE, JACOBIAN] = PULLEY(...) also returns the Jacobian at the estimated
%   pose.
%   
%   Inputs:
%   
%   CABLELENGTH:        1xM vector of cable lengths as given by measurement of
%       the inverse kinematics.
% 
%   PULLEYPOSITION:     Matrix of pulley positions w.r.t. the world frame. Each
%       pulley has its own column and the rows are the x, y, and z-value,
%       respectively i.e., PULLEYPOSITIONS must be a matrix of 3xM values. The
%       number of pulleyvs i.e., N, must match the number of cable attachment
%       points in CABLEATTACHMENT (i.e., its column count) and the order must
%       match the real linkage of pulley to cable attachment on the platform.
% 
%   CABLEATTACHMENT:    Matrix of cable attachment points w.r.t. the platforms
%       platforms coordinate system. Each attachment point has its own column
%       and the rows are the x, y, and z-value, respectively, i.e.,
%       CABLEATTACHMENT must be a matrix of 3xM values. The number of cables
%       i.e., N, must match the number of pulleys in PULLEYPOSITION (i.e., its
%       column count) and the order must match the real linkage of cable
%       attachment on the platform to pulley.
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
%       interpretation of
%   
%       pose = [x_e, y_e, z_e, R11_e, R12_e, R13_e, R21_e, R22_e, R23_e, R31_e,
%       R32_e, R33_e]
%
%   SEE: EUL2ROTM



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-10-08
% Changelog:
%   2016-10-08
%       * Move into package '+fk'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%       * Add assertion through `validateattributes`
%   2016-09-19
%       * Rename to `pulley`
%       * Code cleanup
%   2016-05-01
%       * Update to using EUL2ROTM(eul, 'ZYX') for rotation matrix determination
%   2016-03-30
%       * Code cleanup
%   2015-08-05
%       * Initial release


%% Argument processing
% Five or six input arguments
narginchk(5, 6);
% Zero to two output arguments
nargoutchk(0, 2);

% Default solver options
if nargin < 6 || isempty(SolverOptions)
    SolverOptions = struct();
end

% Assertion of arguments
validateattributes(CableLength, {'numeric'}, {'nonempty', 'vector', 'numel', 12}, mfilename, 'CableLength', 1);
validateattributes(PulleyPosition, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'PulleyPosition', 2);
validateattributes(CableAttachment, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'CableAttachment', 3);
validateattributes(PulleyRadius, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', nume(CableLength)}, mfilename, 'PulleyRadius', 4);
validateattributes(PulleyOrientation, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', nume(CableLength)}, mfilename, 'PulleyOrientation', 5);
validateattributes(SolverOptions, {'struct'}, {'nonempty'}, mfilename, 'SolverOptions', 6);



%% Initialize variables
% Get the provided cable length
vCableLength = asrow(CableLength);
% Get the provided pulley positions
aPulleyPosition = PulleyPosition;
% Get the provided cable attachment points on the platform
aCableAttachment = CableAttachment;
% Get the vector of pulley radius
vPulleyRadius = PulleyRadius;
% Get the matrix of pulley orientations
aPulleyOrientation = PulleyOrientation;
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
opSolverOptions.TolFun = 1e-10;
opSolverOptions.TolX = 1e-12;

% Given any user-defined solver options? Process them now
if numel(stSolverOptionsGiven)
    % Get the fields of the struct provided
    ceFields = fieldnames(stSolverOptionsGiven);
    % And assign each given value to the solver options
    for iField = 1:numel(ceFields)
        % For now we will skip the Jacobian option because it does not work for
        % the Jacobian determined isn't correct
        if strcmpi(ceFields{iField}, 'Jacobian')
            continue
        end
        
        opSolverOptions.(ceFields{iField}) = stSolverOptionsGiven.(ceFields{iField});
    end
end

% Optimization target function
inOptimizationTargetVectorFunction = @(vEstimatedPose) pulley_TargetFunction(vEstimatedPose, vCableLength, aPulleyPosition, aCableAttachment, vPulleyRadius, aPulleyOrientation);

% And now finally run the optimization
[xFinal, resnorm, residual, exitflag, output, lambda, jacobian] = lsqnonlin(inOptimizationTargetVectorFunction ...
    , vInitialStateForOptimization ... % Initial state
    , [], [] ... % lower and upper boundaries are not set
    , opSolverOptions ... % Custom solver options
);



%% Post processing of the estimated pose
% Extract the position
vPosition = xFinal(1:3);
% Extract the quaternion rotation and ...
% vRotation = xFinal(4:7);
% Extract the yaw-pitch-roll rotation angles and ...
vRotation = xFinal(4:6);
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


function [VectorValuedFunction, Jacobian] = pulley_TargetFunction(EstimatedPose, TargetCableLength, PulleyPosition, CableAttachment, PulleyRadius, PulleyOrientation)

%% Preparing the input variables
% Number of cables
nNumberOfCables = size(PulleyPosition, 2);
% Vector of the estimated pose
vEstimatedPose = reshape(EstimatedPose, 1, 6);
% Vector of the target cable lengths
vTargetCableLength = TargetCableLength;
% Get the provided pulley positions
aPulleyPosition = PulleyPosition;
% Get the provided cable attachment points on the platform
aCableAttachment = CableAttachment;
% Get the vector of pulley radius
vPulleyRadius = PulleyRadius;
% Get the matrix of pulley orientations
aPulleyOrientation = PulleyOrientation;
% Extract the position
vPosition = vEstimatedPose(1:3);
% And rotation from the estimated pose
% vRotation = vEstimatedPose(4:7).';
vRotation = vEstimatedPose(4:6).';
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
% Calculate the cable lengths for the estimated pose using the simple
% inverse kinematics algorithm
vLengths = ik_pulley(vEstimatedPose, aPulleyPosition, aCableAttachment, vPulleyRadius, aPulleyOrientation);



%% And build the target optimization vector
% Get the vector difference of all cable lengths ...
vEvaluatedFunction = vLengths(:).^2 - vTargetCableLength(:).^2;

% Also calculate the Jacobian?
if nargout > 1
    % Code taken from WireCenter, therefore not super beautiful and not
    % following code conventions either, but for now it must work
    t1 = cos(vRotation(2));
    t2 = cos(vRotation(3));
    t3 = t1 * t2;
    t5 = sin(vRotation(3));
    t6 = t1 * t5;
    t8 = sin(vRotation(2));
    t12 = cos(vRotation(1));
    t13 = t12 * t8;
    t15 = sin(vRotation(1));
    t17 = -t13 * t2 + t15 * t5;
    t21 = t13 * t5 + t15 * t2;
    t23 = t12 * t1;
    t29 = t15 * t8;
    t32 = t29 * t2 + t12 * t5;
    t36 = -t29 * t5 + t12 * t2;
    t38 = t15 * t1;
    for iCable = 1:nNumberOfCables
        rhom = vPulleyRadius(iCable);
        aPulleyRotation = eul2rotm(fliplr(asrow(deg2rad(aPulleyOrientation(1:3,iCable)))), 'ZYX');
        aPulleyRotation(abs(aPulleyRotation) < 2*eps) = 0;

        t4 = t3 * aCableAttachment(1,iCable);
        t7 = t6 * aCableAttachment(2,iCable);
        t9 = t8 * aCableAttachment(3,iCable);
        t10 = vPosition(1) + t4 - t7 + t9 - aPulleyPosition(1,iCable);
        t18 = t17 * aCableAttachment(1,iCable);
        t22 = t21 * aCableAttachment(2,iCable);
        t24 = t23 * aCableAttachment(3,iCable);
        t25 = vPosition(3) + t18 + t22 + t24 - aPulleyPosition(3,iCable);
        t27 = aPulleyRotation(3,3) * t10 - aPulleyRotation(1,3) * t25;
        t33 = t32 * aCableAttachment(1,iCable);
        t37 = t36 * aCableAttachment(2,iCable);
        t39 = t38 * aCableAttachment(3,iCable);
        t40 = vPosition(2) + t33 + t37 - t39 - aPulleyPosition(2,iCable);
        t43 = aPulleyRotation(1,3) * t40 - aPulleyRotation(2,3) * t10;
        t45 = t27 * aPulleyRotation(3,3) - t43 * aPulleyRotation(2,3);
        t46 = abs(t45);
        t47 = t46 * t46;
        t51 = aPulleyRotation(2,3) * t25 - aPulleyRotation(3,3) * t40;
        t53 = -t43 * aPulleyRotation(1,3) + t51 * aPulleyRotation(3,3);
        t54 = abs(t53);
        t55 = t54 * t54;
        t58 = -t51 * aPulleyRotation(2,3) + t27 * aPulleyRotation(1,3);
        t59 = abs(t58);
        t60 = t59 * t59;
        t61 = t47 + t55 + t60;
        t62 = sqrt(t61);
        t64 = rhom / t62;
        t66 = -vPosition(1) - t4 + t7 - t9 + aPulleyPosition(1,iCable) + t64 * t45;
        t67 = abs(t66);
        t68 = t67 * t67;
        t70 = vPosition(2) + t33 + t37 - t39 - aPulleyPosition(2,iCable) + t64 * t53;
        t71 = abs(t70);
        t72 = t71 * t71;
        t74 = vPosition(3) + t18 + t22 + t24 - aPulleyPosition(3,iCable) + t64 * t58;
        t75 = abs(t74);
        t76 = t75 * t75;
        t77 = rhom * rhom;
        t78 = t68 + t72 + t76 - t77;
        t79 = sqrt(t78);
        t80 = t68 + t72 + t76;
        t81 = sqrt(t80);
        t82 = 1 / t81;
        t84 = acos(t79 * t82);
        t88 = aPulleyRotation(1,3) * t10 + t40 * aPulleyRotation(2,3) + t25 * aPulleyRotation(3,3);
        t90 = acos(t88 * t82);
        t93 = (t84 + t90) * rhom + t79;
        t94 = 1 / t79;
        t95 = t94 * t82;
        t96 = (t66 > 0) - (t66 < 0);
        t97 = t67 * t96;
        t100 = rhom / t62 / t61;
        t101 = (t45 > 0) - (t45 < 0);
        t102 = t46 * t101;
        t103 = aPulleyRotation(3,3).^2;
        t104 = aPulleyRotation(2,3).^2;
        t105 = t103 + t104;
        t107 = (t53 > 0) - (t53 < 0);
        t108 = t54 * t107;
        t109 = aPulleyRotation(1,3) * aPulleyRotation(2,3);
        t111 = (t58 > 0) - (t58 < 0);
        t112 = t59 * t111;
        t113 = aPulleyRotation(1,3) * aPulleyRotation(3,3);
        t115 = t102 * t105 + t108 * t109 + t112 * t113;
        t122 = (t70 > 0) - (t70 < 0);
        t123 = t71 * t122;
        t127 = t64 * t109;
        t130 = (t74 > 0) - (t74 < 0);
        t131 = t75 * t130;
        t135 = t64 * t113;
        t138 = t97 * (-1 - t100 * t45 * t115 + t64 * t105) + t123 * (-t100 * t53 * t115 + t127) + t131 * (-t100 * t58 * t115 + t135);
        t141 = 1 / t81 / t80;
        t142 = t79 * t141;
        t145 = 1 / t80;
        t148 = sqrt(1 - t78 * t145);
        t149 = 1 / t148;
        t152 = t88 * t141;
        t156 = t88 * t88;
        t159 = sqrt(1 - t156 * t145);
        t160 = 1 / t159;
        t170 = aPulleyRotation(1,3).^2;
        t171 = -t170 - t103;
        t173 = aPulleyRotation(3,3) * aPulleyRotation(2,3);
        t175 = -t102 * t109 + t108 * t171 + t112 * t173;
        t190 = t64 * t173;
        t193 = t97 * (-t100 * t45 * t175 - t127) + t123 * (1 - t100 * t53 * t175 + t64 * t171) + t131 * (-t100 * t58 * t175 + t190);
        t212 = -t104 - t170;
        t214 = -t102 * t113 + t108 * t173 + t112 * t212;
        t231 = t97 * (-t100 * t45 * t214 - t135) + t123 * (-t100 * t53 * t214 + t190) + t131 * (1 - t100 * t58 * t214 + t64 * t212);
        t248 = t33 + t37 - t39;
        t251 = -t17 * aCableAttachment(1,iCable);
        t252 = -t21 * aCableAttachment(2,iCable);
        t253 = t251 + t252 - t24;
        t256 = -aPulleyRotation(1,3) * t248 * aPulleyRotation(3,3) - aPulleyRotation(1,3) * t253 * aPulleyRotation(2,3);
        t261 = aPulleyRotation(2,3) * t248 - aPulleyRotation(3,3) * t253;
        t263 = -t170 * t253 + t261 * aPulleyRotation(3,3);
        t267 = -t261 * aPulleyRotation(2,3) - t170 * t248;
        t269 = t102 * t256 + t108 * t263 + t112 * t267;
        t288 = t97 * (-t100 * t45 * t269 + t64 * t256) + t123 * (t251 + t252 - t24 - t100 * t53 * t269 + t64 * t263) + t131 * (t33 + t37 - t39 - t100 * t58 * t269 + t64 * t267);
        t309 = t8 * t2 * aCableAttachment(1,iCable);
        t311 = t8 * t5 * aCableAttachment(2,iCable);
        t312 = t1 * aCableAttachment(3,iCable);
        t313 = -t309 + t311 + t312;
        t315 = t2 * aCableAttachment(1,iCable);
        t316 = t23 * t315;
        t317 = t5 * aCableAttachment(2,iCable);
        t318 = t23 * t317;
        t319 = t13 * aCableAttachment(3,iCable);
        t320 = -t316 + t318 - t319;
        t322 = aPulleyRotation(3,3) * t313 - aPulleyRotation(1,3) * t320;
        t324 = t38 * t315;
        t325 = t38 * t317;
        t326 = t29 * aCableAttachment(3,iCable);
        t327 = t324 - t325 + t326;
        t330 = aPulleyRotation(1,3) * t327 - aPulleyRotation(2,3) * t313;
        t332 = t322 * aPulleyRotation(3,3) - t330 * aPulleyRotation(2,3);
        t337 = aPulleyRotation(2,3) * t320 - aPulleyRotation(3,3) * t327;
        t339 = -t330 * aPulleyRotation(1,3) + t337 * aPulleyRotation(3,3);
        t343 = -t337 * aPulleyRotation(2,3) + t322 * aPulleyRotation(1,3);
        t345 = t102 * t332 + t108 * t339 + t112 * t343;
        t364 = t97 * (t309 - t311 - t312 - t100 * t45 * t345 + t64 * t332) + t123 * (t324 - t325 + t326 - t100 * t53 * t345 + t64 * t339) + t131 * (-t316 + t318 - t319 - t100 * t58 * t345 + t64 * t343);
        t385 = t6 * aCableAttachment(1,iCable);
        t386 = t3 * aCableAttachment(2,iCable);
        t387 = -t385 - t386;
        t389 = t21 * aCableAttachment(1,iCable);
        t390 = -t17 * aCableAttachment(2,iCable);
        t391 = t389 + t390;
        t393 = aPulleyRotation(3,3) * t387 - aPulleyRotation(1,3) * t391;
        t395 = t36 * aCableAttachment(1,iCable);
        t396 = -t32 * aCableAttachment(2,iCable);
        t397 = t395 + t396;
        t400 = aPulleyRotation(1,3) * t397 - aPulleyRotation(2,3) * t387;
        t402 = t393 * aPulleyRotation(3,3) - t400 * aPulleyRotation(2,3);
        t407 = aPulleyRotation(2,3) * t391 - aPulleyRotation(3,3) * t397;
        t409 = -t400 * aPulleyRotation(1,3) + t407 * aPulleyRotation(3,3);
        t413 = -t407 * aPulleyRotation(2,3) + t393 * aPulleyRotation(1,3);
        t415 = t102 * t402 + t108 * t409 + t112 * t413;
        t434 = t97 * (t385 + t386 - t100 * t45 * t415 + t64 * t402) + t123 * (t395 + t396 - t100 * t53 * t415 + t64 * t409) + t131 * (t389 + t390 - t100 * t58 * t415 + t64 * t413);
        aJacobian(iCable,1) = 2 * t93 * ((-(2 * t95 * t138 - 2 * t142 * t138) * t149 / 2 - (aPulleyRotation(1,3) * t82 - t152 * t138) * t160) * rhom + t94 * t138);
        aJacobian(iCable,2) = 2 * t93 * ((-(2 * t95 * t193 - 2 * t142 * t193) * t149 / 2 - (aPulleyRotation(2,3) * t82 - t152 * t193) * t160) * rhom + t94 * t193);
        aJacobian(iCable,3) = 2 * t93 * ((-(2 * t95 * t231 - 2 * t142 * t231) * t149 / 2 - (aPulleyRotation(3,3) * t82 - t152 * t231) * t160) * rhom + t94 * t231);
        aJacobian(iCable,4) = 2 * t93 * ((-(2 * t95 * t288 - 2 * t142 * t288) * t149 / 2 - ((t253 * aPulleyRotation(2,3) + t248 * aPulleyRotation(3,3)) * t82 - t152 * t288) * t160) * rhom + t94 * t288);
        aJacobian(iCable,5) = 2 * t93 * ((-(2 * t95 * t364 - 2 * t142 * t364) * t149 / 2 - ((aPulleyRotation(1,3) * t313 + t327 * aPulleyRotation(2,3) + t320 * aPulleyRotation(3,3)) * t82 - t152 * t364) * t160) * rhom + t94 * t364);
        aJacobian(iCable,6) = 2 * t93 * ((-(2 * t95 * t434 - 2 * t142 * t434) * t149 / 2 - ((aPulleyRotation(1,3) * t387 + t397 * aPulleyRotation(2,3) + t391 * aPulleyRotation(3,3)) * t82 - t152 * t434) * t160) * rhom + t94 * t434);
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

function [Length, CableUnitVectors, PulleyAngles, PulleyPositionCorrected] = pulley(Pose, PulleyPositions, CableAttachments, PulleyRadius, PulleyOrientations)%#codegen
% PULLEY Determine cable lengths based on pulley kinematics
%   
%   Inverse kinematics means to determine the values for the joint variables (in
%   this case cable lengths) for a given endeffector pose. This is quite a
%   simple setup for cable-driven parallel robots because only the equation for
%   the kinematic loop has to be solved, which is the sole purpose of this
%   method. It can determine the cable lengths for any given robot configuration
%   (note that calculations will be done as if we were looking at a 3D/6DOF
%   cable robot following necessary conventions, so adjust your variables
%   accordingly).To determine the cable lengths, both the simple kinematic loop
%   can be used as well as the advanced pulley kinematics (considering pulley
%   radius and rotatability).
% 
%   LENGTH = PULLEY(POSE, PULLEYPOSITIONS, CABLEATTACHMENTS, PULLEYRADIUS,
%   PULLEYORIENTATIONS) performs pulley-based inverse kinematics with the cables
%   running from a_i to b_i for the given pose.
% 
%   [LENGTH, CABLEUNITVECTORS] = PULLEY(...) also provides the the unit
%   vectors for each cable which might come in handy at times.
% 
%   [LENGTH, CABLEUNITVECTORS, PULLEYANGLES] = PULLEY(...) returns the two
%   angles gamma (in first row) and beta (in second row) of the rotation of the
%   pulley about its z-axis and the wrap angle of the cable along the pulley.
% 
%   [LENGTH, CABLEUNITVECTORS, PULLEYANGLES, PULLEYPOSITIONCORRECTED] =
%   PULLEY(...) additionally returns the corrected pulley positions.
%   
%   Inputs:
%   
%   POSE:                   The current robot pose which can be given in three
%       different parametrization options. 1) Given as [x, y, z, a, b, c] where
%       [x, y, z] is the linear position and [a, b, c] are Euler angles in
%       Radian such that a is rotation about x, b about y, and c about z. The
%       resulting rotation order is derived from Tait-Bryan angles i.e., ZYX
%       (first a about x, then b about y, then c about z). 2) Given as [x, y, z,
%       qw, qx, qy, qz] where [x, y, z] is the linear position and [qw, qx, qy,
%       qz] is a quaternion representing the current rotation 3) Given as [x, y,
%       z, R11, R12, R13, R21, R22, R23, R31, R32, R33] where [x, y, z] is the
%       linear position and [R11, R12, R13, R21, R22, R23, R31, R32, R33] is the
%       rotation matrix.
% 
%   PULLEYPOSITIONS:        Matrix of pulley positions w.r.t. the world frame.
%       Each pulley has its own column and the rows are the x, y, and z-value,
%       respective respectively i.e., PULLEYPOSITIONS must be a matrix of 3xM
%       values. The numbe number of pulleys i.e., N, must match the number of
%       cable attachment points in CABLEATTACHMENTS (i.e., its column count) and
%       the order must match the real linkage of pulley to cable attachment on
%       the platform.
% 
%   CABLEATTACHMENTS:       Matrix of cable attachment points w.r.t. the
%       platform's coordinate system. Each attachment point has its own column
%       and the rows are the x, y, and z-value, respectively, i.e.,
%       CABLEATTACHMENTS must be a matrix of 3xM values. The number of cables
%       i.e., N, must match the number of pulleys in PULLEYPOSITIONS (i.e., its
%       column count) and the order must match the real linkage of cable
%       attachment on the platform to pulley.
%
%   PULLEYRADIUS:           1xM matrix of pulley radius for each pulley.
%
%   PULLEYORIENTATIONS:     3xM matrix of orientations of the pulley wrt the
%       inertial reference frame. The transformation will be done using
%       Tait-Bryan XYZ angles in the ZYX convention of ABC (first A about X,
%       then B about Y, finally C about Z).
% 
%   Outputs:
% 
%   LENGTH:                     Length is a vector of size 1xM with the cable
%       lengths determined using pulley-based inverse kinematics.
%   
%   CABLEUNITVECTOR:            Normalized vector for each cable from attachment
%       point to its corrected pulley point as 3xM matrix.
%   
%   PULLEYPOSITIONCORRECTED:    Matrix of 3xM corrected pulley positions i.e.,
%       the point where the cable leaves the pulley and goes to the platform.
%   
%   PULLEYANGLES:               Matrix of gamma and beta angles of rotation and
%       wrapping angle of pulley and cable on pulley, respectively, given as 2xM
%       matrix where the first row is the rotation about the z-axis of the
%       pulley, and the second row is the wrapping angle about the pulley.
%
%   SEE: EUL2ROTM



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-01-05
% Changelog:
%   2017-01-05
%       * Fix calling deprecated `ascolumn` to `ascol`
%   2016-10-08
%       * Move into package '+ik'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%       * Move from `assert` to `validateattributes`
%   2016-09-19
%       * Rename to `pulley`
%   2016-05-23
%       * Update help doc to reflect introduced paramterization option for
%       pose
%   2016-05-20
%       * Allow rotation to be given with Euler Angles ZYX, Quaternion, or
%       a rotation matrix
%   2016-05-01
%       * Update to using EUL2ROTM(eul, 'ZYX') instead of custom-built
%       rotation matrix generation function
%       * Add check to cancel out numeric issues in determination of
%       rotation matrices such that R(abs(R) < 2*eps) = 0
%   2016-04-18
%       * Unification of variable names and matching with new convention by
%       PTT
%       * Add missing help doc for input parameters PULLEYRADIUS and
%       PULLEYORIENTATIONS
%   2016-03-29
%       * Vectorize code to provide a slightly improved execution time
%       (down by 0.001s)
%   2015-08-31
%       * Remove code for shape generation and put into a separate function
%       called algoCableShape_Pulley
%   2015-08-19
%       * Add support for code generation
%   2015-06-24:
%       * Fix incorrect help block
%       * Remove output value CABLEVECTOR
%       * Add output value CABLESHAPE
%   2015-06-18:
%       * Change doc code to align with 80 chars max line length
%       * Make code match PTT's coding convetions
%   2015-04-22:
%       * Initial release



%% Argument processing
% Four or five input arguments
narginchk(4, 5);
% Zero to four output arguments
nargoutchk(0, 4);

% Default pulley orientations
if nargin < 5 || isempty(PulleyOrientations)
    PulleyOrientations = zeros(3, size(PulleyPositions, 2));
end

% Convert pose to row array
Pose = asrow(Pose);
% Check if pose is given with Tait-Bryant angles
if numel(Pose) == 6
    Pose(4:12) = rotm2row(eul2rotm(fliplr(Pose(4:6)./180*pi), 'ZYX'));
% Check if pose is given with quaternion
elseif numel(Pose) == 7
    Pose(4:12) = rotm2row(quat2rotm(quatnormalize(Pose(4:7))));
end

% Assertion of arguments
validateattributes(Pose, {'numeric'}, {'nonempty', 'vector', 'numel', 12}, mfilename, 'Pose', 1);
validateattributes(PulleyPositions, {'numeric'}, {'nonempty', '2d', 'nrows', 3}, mfilename, 'PulleyPositions', 2);
validateattributes(CableAttachments, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', size(PulleyPositions, 2)}, mfilename, 'CableAttachments', 3);
validateattributes(PulleyRadius, {'numeric'}, {'nonempty', 'vector', 'ncols', size(PulleyPositions, 2)}, mfilename, 'PulleyRadius', 4);
validateattributes(PulleyOrientations, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', size(PulleyOrientations, 2)}, mfilename, 'PulleyOrientations', 5);



%% Initialize variables
% To unify variable names
aCableAttachments = CableAttachments;
aPulleyPositions = PulleyPositions;
aPulleyPositionsCorrected = aPulleyPositions;
vPulleyRadius = PulleyRadius;
aPulleyOrientations = PulleyOrientations;
% Get the number of wires
nNumberOfCables = size(aPulleyPositions, 2);
% Holds the actual cable vector
aCableVector = zeros(3, nNumberOfCables);
% Holds the normalized cable vector
aCableVectorUnit = zeros(3, nNumberOfCables);
% Holds the cable lengths
vCableLength = zeros(nNumberOfCables, 1);
% Holds offset to the cable lengths
vCableLengthOffset = zeros(1, nNumberOfCables);
% Extract the position from the pose
vPlatformPosition = ascol(Pose(1:3));
% Extract rotation given as row'ed Rotation matrix from Pose
aPlatformRotation = rotrow2m(Pose(4:12));
% Holds the rotation angle gamma and the wrapping angle beta
aPulleyAngles = zeros(2, nNumberOfCables);



%% Do the magic
% Loop over every pulley and calculate the corrected pulley position i.e.,
% a_{i, corr}
for iUnit = 1:nNumberOfCables
    % Rotation matrix to rotate any vector given in pulley coordinate system
    % K_P into the global coordinate system K_O
    aRotation_kW2kO = eul2rotm(fliplr(asrow(deg2rad(aPulleyOrientations(:,iUnit)))), 'ZYX');
    aRotation_kW2kO(abs(aRotation_kW2kO) < 2*eps) = 0;

    % Vector from contact point of cable on pulley A to cable attachment point
    % on the platform B given in coordinates of system A
    v_W2B_in_kW = transpose(aRotation_kW2kO)*(vPlatformPosition + aPlatformRotation*aCableAttachments(:,iUnit) - aPulleyPositions(:,iUnit));

    % Determine the angle of rotation of the pulley to have the pulley's x-axis
    % point in the direction of the cable which points towards B
    aPulleyAngles(1,iUnit) = atan2d(v_W2B_in_kW(2), v_W2B_in_kW(1));

    % Rotation matrix from winch coordinate system K_W to roller coordinate 
    % system K_R
    aRotation_kR2kW = rotz(aPulleyAngles(1,iUnit));
    aRotation_kR2kW(abs(aRotation_kR2kW) < 2*eps) = 0;

    % Vector from point P (center of coordinate system K_R) to the cable
    % attachment point B given in the coordinate system of the pulley (easily
    % transferable from the same vector given in K_W by simply rotating it about
    % the local z-axis of K_W)
    v_W2B_in_kR = transpose(aRotation_kR2kW)*v_W2B_in_kW;

    % Vector from W to the pulley center given in the pulley coordinate system
    % K_P
    v_W2M_in_kR = vPulleyRadius(iUnit)*[1; 0; 0];

    % Closed vector loop to determine the vector from M to B in coordinate
    % system K_P: P2M + M2B = P2B. This basically also transforms our coordinate
    % system K_P to K_M
    v_M2B_in_kR = v_W2B_in_kR - v_W2M_in_kR;

    % Convert everything in to the coordinate system K_M of the
    % pulley's center
%     v_M2B_in_kM = v_M2B_in_kC;

    % Preliminarily determine the cable length (this helps us to determine the
    % angle beta_3 to later on determine the angle of the vector from M to C in
    % the coordinate system of M. It is quite simple to do so using Pythagoras:
    % l^2 + radius^2 = M2B^2
    dCableLength_C2B = sqrt(norm(v_M2B_in_kR)^2 - vPulleyRadius(iUnit)^2);

    % Determine the angle of rotation of that vector relative to the x-axis of
    % K_P. This is beta_2 in PTT's sketch
    dAngleBetween_M2B_and_xR_Degree = atan2d(v_M2B_in_kR(3), v_M2B_in_kR(1));

    % Now we can determine the angle between M2B and M2C using trigonometric
    % functions because cos(beta_3) = radius/M2B and as well sin(beta_3) = L/M2B
    % or tan(beta_3) = L/radius
    dAngleBetween_M2B_and_M2Ac_Degree = atand(dCableLength_C2B/vPulleyRadius(iUnit));

    % Angle between the x-axis of M and the vector from M to Ac given in
    % coordinate system K_M and in degree
    dAngleBetween_xR_and_M2Ac_Degree = dAngleBetween_M2B_and_M2Ac_Degree + dAngleBetween_M2B_and_xR_Degree;

    % Vector from pulley center M to adjusted cable release point Ac in nothing
    % but the x-axis rotated by the angle beta about the y-axis of K_M
    v_M2Ac_in_kR = transpose(roty(dAngleBetween_xR_and_M2Ac_Degree))*(vPulleyRadius(iUnit).*[1; 0; 0]);

    % Wrapping angle can be calculated in two ways, either by getting the angle
    % between the scaled negative x-axis (M to W) and the vector M to Ac, or by
    % getting the angle between the scaled positive x-axis and the vector M to
    % Ac
    v_M2W_in_kM = vPulleyRadius(iUnit).*[-1; 0; 0];
    dAngleWrap_Degree = acosd(dot(v_M2W_in_kM, v_M2Ac_in_kR)/(norm(v_M2W_in_kM)*norm(v_M2Ac_in_kR)));
    aPulleyAngles(2,iUnit) = dAngleWrap_Degree;

    % Adjust the pulley position given the coordinates to point C
    aPulleyPositionsCorrected(:,iUnit) = aPulleyPositions(:,iUnit) + aRotation_kW2kO*(aRotation_kR2kW*(v_W2M_in_kR + v_M2Ac_in_kR));
    vCableLengthOffset(iUnit) = dAngleWrap_Degree*pi/180*vPulleyRadius(iUnit);
    
    % Finally, calculate the cable vector (from corrected pulley position to the
    % platform)
    aCableVector(:,iUnit) = aPulleyPositionsCorrected(:,iUnit) - ( vPlatformPosition + aPlatformRotation*aCableAttachments(:,iUnit) );
    % And its calculate the cable length
    vCableLength(iUnit) = norm(aCableVector(:,iUnit)) + vCableLengthOffset(iUnit);
    % Last but not least Calculate the direction of the unit vector of the
    % current cable
    aCableVectorUnit(:,iUnit) = aCableVector(:,iUnit)./norm(aCableVector(:,iUnit));
end



%% Output parsing
% First output is the cable lengths
Length = vCableLength;

%%% Further outputs as requested
% Second output is the matrix of normalized cable vectors
if nargout >= 2
    CableUnitVectors = aCableVectorUnit;
end

% Third output is the corrected pulley anchor points
if nargout >= 3
    PulleyAngles = aPulleyAngles;
end

% Fifth output is the corrected pulley angles
if nargout >= 4
    PulleyPositionCorrected = aPulleyPositionsCorrected;
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

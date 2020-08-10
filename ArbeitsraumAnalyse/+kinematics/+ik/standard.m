function [Length, CableUnitVectors, PulleyAngles] = standard(Pose, PulleyPositions, CableAttachments, MotionPattern)%#codegen
% STANDARD Determine cable lengths based on standard kinematics
%   
%   Inverse kinematics means to determine the values for the joint variables (in
%   this case cable lengths) for a given endeffector pose. This is quite a
%   simple setup for cable-driven parallel robots because the equation for the
%   kinematic loop has to be solved, which is the sole purpose of this method.
%   It can determine the cable lengths for any given robot configuration (note
%   that calculations will be done as if we were looking at a 3D/6DOF cable
%   robot following necessary conventions, so adjust your variables
%   accordingly). To determine the cable lengths, both the standard kinematic
%   loop can be used as well as the advanced pulley kinematics (considering
%   pulley radius and rotatability).
% 
%   LENGTH = STANDARD(POSE, PULLEYPOSITIONS, CABLEATTACHMENTS) performs standard
%   inverse kinematics with the cables running from a_i to b_i for the given
%   pose.
%
%   LENGTH = STANDARD(POSE, PULLEYPOSITIONS, CABLEATTACHMENTS, MOTIONPATTERN)
%   specifies the motion pattern to be used. Defaults to 3R3T. Supported motion
%   patterns are
%       1T
%       2T
%       3T
%       1R2T
%       2R3T
%       3R3T
% 
%   [LENGTH, CABLEUNITVECTOR] = STANDARD(...) also provides the unit vectors for
%   each cable which might come in handy at times because it provides the
%   direction of the force impinged by the cable on the mobile platform.
% 
%   [LENGTH, CABLEUNITVECTOR, PULLEYANGLES] = STANDARD(...) additionally returns
%   the angle of rotation that the cable local frame has relative to the world
%   frame's z_0.
%   
%   Inputs:
%   
%   POSE:               The current robot pose which can be given in three
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
%   PULLEYPOSITIONS:    Matrix of pulley positions w.r.t. the world frame. Each
%       pulley has its own column and the rows are the x, y, and z-value,
%       respectively i.e., PULLEYPOSITIONS must be a matrix of (2|3)xM values.
%       The number of pulleys i.e., M, must match the number of cable attachment
%       points in CABLEATTACHMENTS (i.e., its column count) and the order must
%       match the real linkage of pulley to cable attachment on the platform.
% 
%   CABLEATTACHMENTS:   Matrix of cable attachment points w.r.t. the platforms
%       coordinate system. Each attachment point has its own column and the rows
%       are the x, y, and z-value, respectively, i.e., CABLEATTACHMENTS must be
%       a matrix of (2|3)xM values. The number of cables i.e., M, must match the
%       number of pulleys in PULLEYPOSITIONS (i.e., its column count) and the
%       order must match the real linkage of cable attachment on the platform to
%       pulley.
%
%   MOTIONPATTERN       Motion pattern of the cable robot to perform IK for.
% 
%   Outputs:
% 
%   LENGTH:             1xM vetor of cable lengths determined using standard
%       kinematics.
%   
%   CABLEUNITVECTOR:    Normalized vector for each cable from attachment point
%       to its corrected pulley point as (2|3)xM matrix.
%   
%   PULLEYANGLES:       Vector of gamma angles of rotation of the cable plane
%       relative to the world frame, given as 1xM vector where the column is the
%       respective pulley.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-09-14
% Changelog:
%   2017-09-14
%       * Add support for different kind of motion patterns
%   2017-08-07
%       * Resize return parameter 'PulleyAngles' to be consistent with other
%       algorithms i.e., 2xM where the second row represents the rotation about
%       the pulley's wire's z-axis
%   2017-01-05
%       * Fix calling deprecated `ascolumn` to `ascol`
%   2016-10-08
%       * Move into package '+ik'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%   2016-09-19
%       * Rename to `standard`
%   2016-09-12
%       * Fix calling deprecated `matnormcols` to `mnormcols`
%   2016-05-23
%       * Update help doc to reflect introduced paramterization option for
%       pose
%   2016-05-20
%       * Allow rotation to be given with Euler Angles ZYX, Quaternion, or
%       a rotation matrix
%   2016-04-04
%       * Add matnormcols for normalization of cable vectors
%   2016-03-29
%       * Code cleanup
%       * Vectorize code to provide a slightly improved execution time
%       (down by 0.001s)
%   2015-08-31
%       * Remove code for shape generation and put into a separate function
%       called algoCableShape_Standard
%   2015-08-19
%       * Add support for code generation
%   2015-06-24
%       * Add return value CABLESHAPE and remove CABLEVECTOR
%       * Add return value PULLEYANGLES
%   2015-04-22
%       * Initial release



%% Argument processing
% Three input arguments
narginchk(3, 4);
% Zero to three output arguments
nargoutchk(0, 3);

% Default motion pattern
if nargin == 3 || 1 ~= exist('MotionPattern', 'var') || isempty(MotionPattern)
    MotionPattern = '3R3T';
end

% Validate the given motion pattern
MotionPattern = upper(validatestring(upper(MotionPattern), {'1t', '2t', '3t', '1r2t', '2r3t', '3r3t'}, mfilename, 'MotionPattern', 4));

% Convert pose to column vector
Pose = Pose(:);

% Make sure the pose is given in a correct format
switch MotionPattern
    case '1T'
        nPose = 1;
        nCart = 1;
    case '2T'
        nPose = 2;
        nCart = 2;
    case '3T'
        nPose = 3;
        nCart = 3;
    case '1R2T'
        nPose = 6;
        nCart = 2;
    case '2R3T'
    case '3R3T'
        nPose = 12;
        nCart = 3;
end

% Assertion of arguments
validateattributes(Pose, {'numeric'}, {'nonempty', 'vector', 'numel', nPose}, mfilename, 'Pose', 1);
validateattributes(PulleyPositions, {'numeric'}, {'nonempty', '2d', 'nrows', nCart}, mfilename, 'PulleyPositions', 2);
validateattributes(CableAttachments, {'numeric'}, {'nonempty', '2d', 'nrows', nCart, 'ncols', size(PulleyPositions, 2)}, mfilename, 'CableAttachments', 3);



%% Initialize variables
% To unify variable names
aCableAttachments = CableAttachments;
aPulleyPositions = PulleyPositions;
nCables = size(aPulleyPositions, 2);
% Extract position and rotation depending on the motion pattern
switch MotionPattern
    case '1T'
        vPlatformPosition = Pose(1);
        aPlatformRotation = zeros(1, 1);
    case '2T'
        vPlatformPosition = Pose(1:2);
        aPlatformRotation = zeros(2, 2);
    case '3T'
        vPlatformPosition = Pose(1:3);
        aPlatformRotation = zeros(3, 3);
    case '1R2T'
        vPlatformPosition = Pose(1:2);
        aPlatformRotation = [Pose(3), Pose(4); Pose(5), Pose(6)];
    case '2R3T'
    case '3R3T'
        vPlatformPosition = Pose(1:3);
        aPlatformRotation = rotrow2m(asrow(Pose(4:12)));
end



%% Do the magic with vectorized code
% Get the matrix of all cable vectors
aCableVector = aPulleyPositions - (repmat(vPlatformPosition, 1, nCables) + aPlatformRotation*aCableAttachments);

% Get the cable lengths (note, ```norm``` doesn't work here because that would
% take the norm of the matrix, however, we want the norm of each column thus
% we're using sqrt(sum(col.^2))
vCableLength = sqrt(sum(aCableVector.^2));



%% Output parsing
% First output is the cable lengths
Length = vCableLength;

% Further outputs as requested
% Second output is the matrix of normalized cable vectors
if nargout > 1
    % Get the unit vectors of each cable
    CableUnitVectors = math.mnormcol(aCableVector);
end

% Third output is the rotation angle of each cable plane but only for three
% dimensional cases
if nargout > 2 && ~isempty(strfind(MotionPattern, '3T'))
    % Calculate the angle of rotation of the cable local frame K_c relative to K_0
    % for all cables at once
    PulleyAngles = zeros(1, nCables);
    PulleyAngles(end,:) = atan2d(-aCableVector(2,:), -aCableVector(1,:));
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

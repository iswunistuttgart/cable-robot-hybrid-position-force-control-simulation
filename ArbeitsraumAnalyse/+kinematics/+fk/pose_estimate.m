function [PoseEstimate] = pose_estimate(CableLength, PulleyPosition, CableAttachment)%#codegen
% POSE_ESTIMATE estimates the initial pose for forward kinematics
% 
%   POSEESTIMATE = POSE_ESTIMATE(CABLELENGTH, PULLEYPOSITION, CABLEATTACHMENT)
%   calculates the center of the bounding box of the given pulley positions and
%   cable attachments.
%   
%   Inputs:
%   
%   CABLELENGTH:        Vector of Mx1 cable lengths for which to estimate the
%       initial pose.
% 
%   PULLEYPOSITIONS:    Matrix of pulley positions w.r.t. the world frame. Each
%       pulley has its own column and the rows are the x, y, and z-value,
%       respectively i.e., PULLEYPOSITIONS must be a matrix of 3xM values. The
%       number of pulleys i.e., N, must match the number of cable attachment
%       points in CABLEATTACHMENTS (i.e., its column count) and the order must
%       match the real linkage of pulley to cable attachment on the platform.
% 
%   CABLEATTACHMENTS:   Matrix of cable attachment points w.r.t. the platforms
%       platforms coordinate system. Each attachment point has its own column
%       and the rows are the x, y, and z-value, respectively, i.e.,
%       CABLEATTACHMENTS must be a matrix of 3xM values. The number of cables
%       i.e., N, must match the number of pulleys in PULLEYPOSITIONS (i.e., its
%       column count) and the order must match the real linkage of cable
%       attachment on the platform to pulley.
% 
%   Outputs:
% 
%   POSEESTIMATE:       Vector of 1x6 values representing the initial position
%       guess in [x, y, z] and the initial rotation guess in [a, b, c]
%       (currently set to 0).



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-10-08
% Changelog:
%   2016-10-08
%       * Move into package '+fk'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%       * Add assertion through `validateattributes`
%   2016-09-19
%       * Rename to `pose_estimate`
%   2016-03-30
%       * Code cleanup
%   2015-08-05
%       * Initial release



%% Argument processing
% Three input arguments
narginchk(3, 3);
% Zero or one output argument
nargoutchk(0, 1);

% Assertion of arguments
validateattributes(CableLength, {'numeric'}, {'nonempty', 'vector', 'numel', 12}, mfilename, 'CableLength', 1);
validateattributes(PulleyPosition, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'PulleyPosition', 2);
validateattributes(CableAttachment, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', numel(CableLength)}, mfilename, 'CableAttachment', 3);



%% Initialize variables
vCableLengths = asrow(CableLength);
aPulleyPositions = PulleyPosition;
aCableAttachments = CableAttachment;
% By default we will estimate to the center of the world coordinate system
% with a so called "zero"-rotation
vPoseEstimate = zeros(6, 1);



%% Calculate the bounding box
vMinRadius = aPulleyPositions(:, 1) - ones(3, 1).*(vCableLengths(1) + norm(aCableAttachments(1)));
vMaxRadius = aPulleyPositions(:, 1) + ones(3, 1).*(vCableLengths(1) + norm(aCableAttachments(1)));

% Loop over each cable and adjust the min and max radius accordingly
for iCable = 1:size(aPulleyPositions, 2)
    vCurrentRadiusMin = aPulleyPositions(:, iCable) - ones(3, 1).*(vCableLengths(iCable) + norm(aCableAttachments(iCable)));
    vCurrentRadiusMax = aPulleyPositions(:, iCable) + ones(3, 1).*(vCableLengths(iCable) + norm(aCableAttachments(iCable)));
    vMinRadius = max(vMinRadius, vCurrentRadiusMin);
    vMaxRadius = min(vMaxRadius, vCurrentRadiusMax);
end

% Use the center of the bounding box as an initial estimate
vPoseEstimate(1:3) = (vMaxRadius + vMinRadius)./2;



%% Assign output quantities
PoseEstimate = vPoseEstimate;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

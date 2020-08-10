function [StructureMatrix, NullSpace] = m2R3T(CableAttachments, CableVectors, Rotation)%#codegen
% M2R3T - Calculate the structure matrix for the given cable attachment
% points and cable vectors of a 3R3T cable robot
% 
%   STRUCTUREMATRIX = M2R3T(CABLEATTACHMENTS, CABLEVECTORS) determines
%   the structure matrix for the given cable attachment points and the given
%   cable vectors. Cable vectors can but must not be a matrix of normalized
%   vectors.
%
%   STRUCTUREMATRIX = M2R3T(CABLEATTACHMENTS, CABLEVECTORS, ROTATION)
%   assumes the platform rotation given by ROTATION.
%
%   [STRUCTUREMATRIX, NULLSPACE] = M2R3T(...) also returns the nullspace
%   of structure matrix STRUCTUREMATRIX.
%   
%   Inputs:
%   
%   CABLEATTACHMENTS:   Matrix of cable attachment points w.r.t. the platforms
%       coordinate system. Each attachment point has its own column and the rows
%       are the x, y, and z-value, respectively, i.e., CABLEATTACHMENTS must be
%       a matrix of 3xM values. The number of cables i.e., N, must match the
%       number of winches in WINCHPOSITIONS (i.e., its column count) and the
%       order must match the real linkage of cable attachment on the platform to
%       winch.
%   
%   CABLEVECTORS:       Matrix of cable direction vectors from CABLEATTACHMENTS
%       to the winch attachment point. Must not be a matrix of normalized
%       values, however, must be a 3xM matrix of coordinates [x, y, z]'.
%
%   ROTATION:           Current rotation of the platform. Can be a 1x3 vector
%       representing Tait-Bryan Euler angles ZYX (yaw = z, pitch = y, roll = x)
%       rotation. May also directly be a rotation matrix of size 3x3. If not
%       provided, unit-rotation will be assumed.
% 
%   Outputs:
% 
%   STRUCTUREMATRIX:    Structure matrix At for the given attachment points
%       given the cable vectors. At is of size 6xM.
%
%   NULLSPACE:          The corresponding nullspace to structure matrix At.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-04-14
% Changelog:
%   2017-04-14
%       * Initial release



%% Argument processing
% Two to three input arguments
narginchk(2, 3);
% Zero to two output arguments
nargoutchk(0, 2);

% Default rotation
if nargin < 3 || isempty(Rotation)
    Rotation = eye(3);
end

% Extract rotation given in Euler angles from Pose
if numel(Rotation) == 3
    Rotation = eul2rotm(fliplr(asrow(Rotation./180*pi)), 'ZYX');
% Extract rotation given as Quaternion from Posae
elseif numel(Rotation) == 4
    Rotation = quat2rotm(quatnormalize(asrow(Rotation)));
% Extract rotation given as row'ed Rotation matrix
elseif numel(Rotation) == 9 && isvector(Rotation)
    Rotation = rotrow2m(Rotation);
end

% Assertion of arguments
validateattributes(CableAttachments, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', size(CableVectors, 2), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'CableAttachments', 1);
validateattributes(CableVectors, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', size(CableAttachments, 2), 'nonnan', 'finite', 'nonsparse'}, mfilename, 'CableVectors', 2);
validateattributes(Rotation, {'numeric'}, {'nonempty', 'square', 'nrows', 3, 'ncols', 3, '>=', -1, '<=', 1, 'nonsparse', 'finite', 'nonnan'}, mfilename, 'Rotation', 3);



%% Parse Variables
% Get number of wires
nNumberOfWires = size(CableAttachments, 2);
% Create the structure matrix's matrix
aStructureMatrix = zeros(6, nNumberOfWires);
% Keeping variable names consistent
aCableVectors = CableVectors;
aCableAttachments = CableAttachments;
aRotation = Rotation;



%% Create the structure matrix
% Normalize all columns in a quick fashion
aCableVectors = aCableVectors./repmat(sqrt(sum(aCableVectors.^2)), 3, 1);

% Force parts are just the cable unit vector directions
aStructureMatrix(1:3,:) = aCableVectors;
% Calculate the torque parts
aStructureMatrix(4:6,:) = cross(aRotation*aCableAttachments, aCableVectors);



%% Assign output quantities
% First output: structure matrix; required
StructureMatrix = aStructureMatrix;

% Second output: nullspace of structure matrix; optional
if nargout > 1
    NullSpace = null(StructureMatrix);
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

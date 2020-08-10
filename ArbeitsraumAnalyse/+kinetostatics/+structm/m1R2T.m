function [StructureMatrix, NullSpace] = m1R2T(CableAttachments, CableVectors, Rotation)%#codegen
% M1R2T - Calculate the structure matrix for the given cable attachment
% points and cable vectors of a 1R2T cable robot
% 
%   STRUCTUREMATRIX = M1R2T(CABLEATTACHMENTS, CABLEVECTORS) determines
%   the structure matrix for the given cable attachment points and the given
%   cable vectors. Cable vectors can but must not be a matrix of normalized
%   vectors.
%
%   STRUCTUREMATRIX = M1R2T(CABLEATTACHMENTS, CABLEVECTORS, ROTATION)
%   assumes the platform rotation given by ROTATION.
%
%   [STRUCTUREMATRIX, NULLSPACE] = M1R2T(...) also returns the nullspace
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
%   ROTATION:           Current rotation of the platform. Can be a scalar
%       representing the rotation about the z-axis, or a 2x2 matrix representing
%       the rotation matrix. If not provided, unit-rotation will be assumed.
% 
%   Outputs:
% 
%   STRUCTUREMATRIX:    Structure matrix A' for the given attachment points
%       given the cable vectors. Is of size 3xM.
%
%   NULLSPACE:          The corresponding nullspace to structure matrix At.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-04-14
% Changelog:
%   2017-04-14
%       * Extend VALIDATEATTRIBUTES calls with a few more safety checks
%       * Remove manual creation of rotation matrix and implement ROT2
%   2016-12-11
%       * Vectorize operations to make them quicker
%       * Fix bug with validateattributes using `nonan` instead of `nonnan`
%   2016-10-19
%       * Remove check for maximum value because it won't work with Simulink
%       code propagation
%   2016-10-08
%       * Move into package '+structm'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%       * Set argument ROTATION to only accept 2x2 matrix input
%   2016-09-19
%       * Rename to `m1R2T`
%   2016-03-30
%       * Add output parameter NULLSPACE
%   2016-03-29
%       * Code cleanup
%   2015-08-19
%       * Fix wrong function name
%       * Fix error with the cross product coming from two 2x1 vectors being put
%       in
%       * Change size of structure matrix to 3xM
%   2015-06-25
%       * Initial release



%% Argument processing
% Two to three input arguments
narginchk(2, 3);
% Zero to two output argument
nargoutchk(0, 2);

% Default rotation: None
if nargin < 3 || isempty(Rotation)
    Rotation = eye(2);
end

% If rotation is given as scalar, we'll make it a matrix (assuming it is given
% in radian)
if numel(Rotation) == 1
    Rotation = rot2(Rotation);
end

% Assertion of arguments
validateattributes(CableAttachments, {'numeric'}, {'nonempty', '2d', 'nrows', 2, 'finite', 'nonnan', 'nonsparse'}, mfilename, 'CableAttachments', 1);
validateattributes(CableVectors, {'numeric'}, {'nonempty', '2d', 'nrows', 2, 'ncols', size(CableAttachments, 2), 'finite', 'nonnan', 'nonsparse'}, mfilename, 'CableVectors', 2);
validateattributes(Rotation, {'numeric'}, {'nonempty', 'square', 'nrows', 2, 'ncols', 2, '>=', -1, '<=', 1, 'nonsparse', 'finite', 'nonnan'}, mfilename, 'Rotation', 3);



%% Parse Variables
% Get number of wires
nNumberOfWires = size(CableAttachments, 2);
% Create the structure matrix's matrix
aStructureMatrix = zeros(3, nNumberOfWires);
% Keeping variable names consistent
aCableVectors = CableVectors;
aCableAttachments = CableAttachments;
aRotation = Rotation;



%% Create the structure matrix
% Normalize all columns in a quick fashion
aCableVectors = aCableVectors./repmat(sqrt(sum(aCableVectors.^2)), 2, 1);

% Force directions are in the first two rows
aStructureMatrix(1:2,:) = aCableVectors(1:2,:);

% Third row is resulting from the torque so it is dot([-uy; ux], -R*[bx; by]) or
% uy*(bx*cos(g) + by*(-sin(g))) - ux*(by*cos(g) + bx*sin(g))
aStructureMatrix(3,:) =   aCableVectors(2,:).*(aCableAttachments(1,:)*aRotation(1,1) + aCableAttachments(2,:).*aRotation(1,2)) + ...
                        - aCableVectors(1,:).*(aCableAttachments(2,:)*aRotation(1,1) + aCableAttachments(1,:).*aRotation(2,1));



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

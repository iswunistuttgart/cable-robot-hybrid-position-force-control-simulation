function [StructureMatrix, NullSpace] = m3T(CableAttachments, CableVectors)%#codegen
% M3T - Calculate the structure matrix for the given cable attachment points and
% cable vectors of a 3T cable robot
% 
%   STRUCTUREMATRIX = M3T(CABLEATTACHMENTS, CABLEVECTORS) determines the
%   structure matrix for the given cable attachment points and the given cable
%   vectors. Cable vectors can but must not be a matrix of normalized vectors.
%
%   [STRUCTUREMATRIX, NULLSPACE] = M3T(...) also returns the nullspace of
%   structure matrix STRUCTUREMATRIX.
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
%   Outputs:
% 
%   STRUCTUREMATRIX:    Structure matrix A' for the given attachment points
%       given the cable vectors. Is of size 3xM.
%
%   NULLSPACE:          The corresponding nullspace to structure matrix At.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-11
% Changelog:
%   2016-12-11
%       * Speed up calculation of structure matrix
%   2016-10-08
%       * Move into package '+structm'
%       * Add `narginchk`, `nargoutchk` for better standalone usage
%   2016-09-19
%       * Rename to `m3T`
%   2016-03-30
%       * Add output parameter NULLSPACE
%   2016-03-29
%       * Code cleanup
%   2015-06-25
%       * Initial release


%% Argument processing
% No more than two input arguments
narginchk(2, 2);
% Zero to two output arguments
nargoutchk(0, 2);

% Assertion of arguments
validateattributes(CableAttachments, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'finite', 'nonnan', 'nonsparse'}, mfilename, 'CableAttachments', 1);
validateattributes(CableVectors, {'numeric'}, {'nonempty', '2d', 'nrows', 3, 'ncols', size(CableAttachments, 2), 'finite', 'nonnan', 'nonsparse'}, mfilename, 'CableVectors', 2);


%% Parse Variables
% Keeping variable names consistent
aCableVectors = CableVectors;



%% Create the structure matrix
% Normalize all columns in a quick fashion
aCableVectors = aCableVectors./repmat(sqrt(sum(aCableVectors.^2)), 3, 1);

% The structure matrix equals the cable vectors as there are no rotations
aStructureMatrix = aCableVectors;


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

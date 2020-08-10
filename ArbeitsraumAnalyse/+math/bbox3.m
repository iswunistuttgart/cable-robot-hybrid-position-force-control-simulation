function [Box, Traversal] = bbox3(X, Y, Z, varargin)
% BBOX3 Calculates the 3D bounding box for the given points
%  
%   BOX = BBOX3(X, Y, Z) calculates the bounding box for the given points in X,
%   Y, Z position and returns a matrix of size 3x8 where each column is one of
%   the bounding box' corners.
%   Basically, what BBOX3 does is take all the mins and max' from the values of
%   X, Y, and Z and assigns them properly into box.
%
%   BOX = BBOX3(MATRIX) extracts the X, Y, and Z data from the Nx3 matrix MATRIX
%   and then gets the bounding box on these values.
%
%   [BOX, TRAVERSAL] = BBOX3(X, Y, Z) also returns the array of traversals which
%   relates to the corners of BOX to get a full patch.
%
%   
%   Inputs:
%   
%   X: Vector or matrix of points on the YZ-plane
%   
%   Y: Vector or matrix of points on the XZ-plane
%
%   Z: Vector or matrix of points on the XY-plane
%
%   Outputs:
%
%   BOX: Matrix of 3x8 entries that correspond the corners of the bounding box
%   with their relation as given in the second output parameter TRAVERSAL
%
%   TRAVERSAL: Matrix of 6 rows of size 3 where each row corresponds to one
%   traversal of the bounding box BOX for using the patch command
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-08-04
% Changelog:
%   2017-08-04
%       * Convert all ```assert``` into ```validateattributes``` for better
%       error display
%   2016-05-30
%       * Update matrix argument processing to column major i.e., [X, Y, Z]
%   2016-05-10
%       * Rename to bbox3
%       * Add possibility to pass a 3xN or Nx3 matrix as only argument
%   2016-04-13
%       * Fix bug on getting the min and max vals of X, Y, and Z
%   2016-04-01
%       * Update file-information block
%       * Update help documentation
%       * Update assertion
%   2014-01-21
%       * Initial release



%% Pre-process input
% If there's only one argument and it is a matrix of size Mx3
if ismatrix(X) && size(X, 2) == 3
    % ... grab Z from X
    Z = X(:,3);
    % ... grab Y from X
    Y = X(:,2);
    % ... and grab X from X
    X = X(:,1);
end



%% Validate inputs
try
    % X must not be a scalar and must be a vector
    validateattributes(X, {'numeric'}, {'vector', 'nonempty', 'finite', 'nonsparse', 'numel', numel(Y)}, mfilename, 'X');
    % Y must not be a scalar and must be a vector
    validateattributes(Y, {'numeric'}, {'vector', 'nonempty', 'finite', 'nonsparse', 'numel', numel(X)}, mfilename, 'Y');
    % Y must not be a scalar and must be a vector
    validateattributes(Z, {'numeric'}, {'vector', 'nonempty', 'finite', 'nonsparse', 'numel', numel(X)}, mfilename, 'Z');
catch me
    throwAsCaller(me);
end



%% Do the magic!
% First, get all minimum and maximum values of X, Y, and Z
vMinVals = [min(X), min(Y), min(Z)];
vMaxVals = [max(X), max(Y), max(Z)];

% Holds our output
aBoundingBox = zeros(8, 3);

% The first set of points will be all points on the lower side of the cube
aBoundingBox(1, :) = [vMinVals(1), vMinVals(2), vMinVals(3)];
aBoundingBox(2, :) = [vMaxVals(1), vMinVals(2), vMinVals(3)];
aBoundingBox(3, :) = [vMaxVals(1), vMaxVals(2), vMinVals(3)];
aBoundingBox(4, :) = [vMinVals(1), vMaxVals(2), vMinVals(3)];
% Second half of points will be all points on the upper side of the cube
aBoundingBox(5, :) = [vMinVals(1), vMinVals(2), vMaxVals(3)];
aBoundingBox(6, :) = [vMaxVals(1), vMinVals(2), vMaxVals(3)];
aBoundingBox(7, :) = [vMaxVals(1), vMaxVals(2), vMaxVals(3)];
aBoundingBox(8, :) = [vMinVals(1), vMaxVals(2), vMaxVals(3)];

% This allows to use results of bbox3 as input to patch('Vertices', box, 'Faces', traversal)
aTraversal = [1, 2, 3, 4; ...
    5, 6, 7, 8; ...
    1, 2, 6, 5; ...
    2, 3, 7, 6; ...
    3, 4, 8, 7; ...
    4, 1, 5, 8];



%% Assign output quantities
% Main output is the bounding box
Box = aBoundingBox;

% First optional output is the array of traversals
if nargout > 1
    Traversal = aTraversal;
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

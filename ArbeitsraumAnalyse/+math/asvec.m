function Y = asvec(X, Dim, Order)
% ASVEC return elements of array X as a vector
%
%   ASCOL(X) returns the elements of X as a column vector, running through the
%   elements of X in column major order.
%
%   ASVEC(X, DIM) returns the elements of X as a vector along dimension DIM. For
%   instance, ASVEC(X, 1) returns the elements of X as a column vector and
%   ASVEC(X, 2) returns the elements as a row vector.
%
%   ASVEC(X, DIM, ORDER) will run through the dimensions of X in the order
%   specified in ORDER.  Unspecified dimensions will be run through in
%   increasing order after the specified dimensions.  For instance, if X is 4-D,
%   then ASVEC(X, DIM, [2 3]) = ASVEC(X, DIM, [2 3 1 4]).  Row major order is
%   obtained with ASVEC(X, DIM, 2).
%
%   ASVEC(X, DIM) is equivalent to ASVEC(X, DIM, 1:NDIMS(X)).
%
%   See also: ASCOL, ASROW, PERMUTE, IPERMUTE.
%
%   Inputs:
%
%   X                    Array of arbitrary size that shall be converted to a
%       vector.
%
%   DIM                  Defines along which dimension the resulting vector
%       should be created.
%
%   ORDER                Allows to specify the order of dimensions that ASVEC
%       will run through.
%
%   Outputs:
%
%   Y                    Vector with all elements of array X in specified order.



%% File information
% Author: Peter J. Acklam <pjacklam@online.no>
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-03
% Changelog:
%   2016-12-03
%       * Initial release



%% Do your code magic here
narginchk(1, 3);

% If dimension order is specified, then reorder the dimension of X
if nargin == 3
    % largest dimension to deal with
    nLargestDim = max(ndims(X), max(Order));
    % one `1' for each dimension
    vRestDim = ones(1, nLargestDim);
    % remove dimension in `order'
    vRestDim(Order) = 0;
    % dimension permutation vector
    vDimPermutation = [Order find(vRestDim)];
    % permute dimensions in `x'
    X = permute(X, vDimPermutation);
end

% Default dimension for output vector is 1 (i.e., column vector)
if nargin < 2
    Dim = 1;
end

% Initialize size vector for `Y`
vSizeY = ones(1, max(2, Dim));

% All elements along dimension `Dim`
vSizeY(Dim) = numel(X);

% Convert `X` into a vector
Y = reshape(X, vSizeY);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

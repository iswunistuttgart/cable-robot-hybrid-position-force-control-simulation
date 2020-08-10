function [N] = nrows(A, NExpected)%#codegen
% NROWS Count number of rows of matrix A
%
%   N = NROWS(A) get the number of rows of 2D matrix A.
%
%   N = NROWS(A, NEXPECTED) checks the number of rows of 2D matrix A against
%   NEXPECTED. If it is the same number, then M is true, otherwise false.
%
%   
%   Inputs:
%   
%   A: Matrix or vector to count the rows of
%
%   MEXPECTED: Expected number of rows
% 
%   Outputs:
% 
%   N: Number of rows of A or true/false if NExpected given and number of rows
%   of A equals NExpected
% 



%% File Information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-04-08
% Changelog:
%   2016-04-08
%       * Replace validateattributes by assert
%       * Update docs
%       * Add narginchk
%   2015-05-10
%       * Update help documentation and add support for variables of type 2d
%   2015-04-27
%       * Initial release



%% Pre-process inputs
narginchk(1, 2);
% Allow for fallback MExpected
if nargin < 2
    NExpected = Inf;
end



%% Assert inputs
% Need a matrix of doubles
assert(isa(A, 'double'), 'Argument [A] must be of type double');
assert(ismatrix(A), 'Argument [A] must be a matrix');
% Make sure we check MExpected only if its given
if ~isinf(NExpected)
    % NExpected must be of type double and must also be greater than zero
    assert(isa(NExpected, 'double'), 'Argument [MExpected] must be of type double');
    assert(NExpected > 0, 'Argument [MExpected] must be greater than zero');
end



%% Magic
% Get the number of rows
nRows = size(A, 1);

% Need to check against something?
if ~isinf(NExpected)
    N = nRows == NExpected;
% Don't check againts a value
else
    % Get number of rows of A
    N = nRows;
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

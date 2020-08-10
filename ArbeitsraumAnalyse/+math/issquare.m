function flag = issquare(A, n)%#codegen
% ISSQUARE - Check whether the given matrix is square
% 
%   FLAG = ISSQUARE(A) checks matrix A is square i.e., has same row and column
%   count or $n == m$.
%
%   FLAG = ISSQUARE(A, N) checks that matrix A is of square size N i.e., has N
%   rows and N columns.
%
% 
%   Inputs:
% 
%   A: matrix to check for squareness
%
%   N: Finite, positive integer to check square size of A against
% 
%   Outputs:
% 
%   FLAG: Logical true if A is square (and of size NxN), otherwise fale
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-12-23
% Changelog:
%   2016-12-23
%       * Add narginchk and nargoutck
%   2016-05-01
%       * Add input argument N
%       * Add new file information block
%       * Add assertion
%       * Add codegen directive
%       * Advance checking of matrix A
%   2015-04-22
%       * Initial release



%% Assert arguments
narginchk(1, 2);
nargoutchk(0, 1);



%% Input parsing
if nargin < 2
    n = Inf;
end



%% Assertion
% Assert that the optional argument n is either inf (set by us) or that it is
% not inf and a positive integer
if ~isinf(n)
    % Positive number required
    assert(n > 0);
    % Integer required
    assert(mod(n, 1) < eps);
end



%% Actual check
% Default check result
flag = false;
% Check A is a matrix and not a vector (otherwise any row or column vector would
% continue to be checked).
if ismatrix(A) && ~isvector(A)
    % Check for a given size
    if ~isinf(n)
        flag = isequal(size(A, 1), size(A, 2), n);
    % Check its square in general
    else
        flag = isequal(size(A, 1), size(A, 2));
    end
end


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changelog" section of the header

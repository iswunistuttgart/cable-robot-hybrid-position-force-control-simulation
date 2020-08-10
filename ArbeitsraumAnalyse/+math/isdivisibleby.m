function f = isdivisibleby(n, d)
% ISDIVISIBLEBY checks if number is divisable by divisor
%
%   F = ISDIVISIBLEBY(N, D) checks whether number N is divisable by number D and
%   returns flag F as true or false.
%
%   Allowed are the following input sizes
%
%    N       D
%   1x1     1xN
%   1xN     1x1
%
%   Inputs:
%
%   N                   1xN array of numbers to check for divisibility.
%
%   D                   1xM array of numbers to check divisibility by for. D
%                       can be a matrix of 1 or N columns, depending on what
%                       should be checked for.
%
%   Outputs:
%
%   F                   MxN array of flags for each number whether N(i,j) is
%                       divisable by D(i,j).



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-08-14
% Changelog:
%   2017-08-14
%       * Initial release



%% Validate input arguments
try
    % Check N
    validateattributes(n, {'numeric'}, {'vector', 'nonempty', 'nonnan', 'nonsparse', 'finite', 'nonzero'}, mfilename, 'N');
    % Check D
    validateattributes(d, {'numeric'}, {'vector', 'nonempty', 'nonnan', 'nonsparse', 'finite', 'nonzero'}, mfilename, 'D');
    
    % Check N and D have similar shape
    assert(isvector(n) && isscalar(d) || isscalar(n) && isvector(d), 'PHILIPPTEMPEL:MATLABTOOLING:DIVISIBLE:InvalidArgument', 'N and D must be of compatible size. N must be 1xN or MxN and D must be 1xN or MxN.');
catch me
    throwAsCaller(me);
end



%% Do your code magic here
% It's that simple
r = rem(n, d);
r(isnan(r)) = 0;
f = ~logical(r);



end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

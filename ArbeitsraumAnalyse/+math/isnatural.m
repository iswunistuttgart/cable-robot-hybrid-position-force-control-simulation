function flag = isnatural(A)
% ISNATURAL True for arrays of positive i.e., non-negative natural nmbers.
%
%   ISNATURAL(A) returns true if A is an array of positive natural numbers.
%
%   Inputs:
%
%       A   Array to be checked for integer value.
%
%   Outputs:
%
%       FLAG    TRUE if all of array is integers (i.e., positive natural
%               numbers), FALSE otherwise.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-06-14
% Changelog:
%   2016-06-14
%       * Initial release



%% Assert arguments
narginchk(1, 1);
nargoutchk(0, 1);



%% Magic, do your thing

flag = all(isint(A(:))) && all(A(:)) > 0;

end
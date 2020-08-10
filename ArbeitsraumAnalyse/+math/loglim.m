function l = loglim(d)
%% LOGLIM returns logarithmic limits
%
%   L = LOGLIM(D) returns logarithmic limits L for the values given in D. The
%   formula is simple:
%   L = [floor( ( log10(q) / 3 - 1 ) * 3), ceil( ( log10(q) / 3 + 1 ) * 3)]
%
%   Inputs:
%
%   D                   NxK array of values to obtain limits for.
%
%   Outputs:
%
%   L                   Nx2 array of lower and upper logarithmic limits.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2019-01-06
% Changelog:
%   2019-01-06
%       * Fix implementation not working correcty for numbers in
%       [1+eps,1000-eps].
%   2018-12-30
%       * Initial release



%% Validate arguments

validateattributes(d, {'numeric'}, {'nonempty', 'finite', 'positive'}, mfilename, 'd');



%% Do your code magic here

% Sort in ascending order
d = sort(d, 2);

% Express numbers in base-10 system
lg = log10(d);

% Just like so (to powers of 10^3)
l = [floor(floor(min(lg, [], 2))./3).*3, ceil(ceil(max(lg, [], 2))./3).*3];


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

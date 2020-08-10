function v = versin(th)
% VERSIN calculates the versine of the argument
%
%   Inputs:
%
%   TH                  NxM array of values to calculate versine of
%
%   Outputs:
%
%   V                   NxM array of versines of Z



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-10-10
% Changelog:
%   2017-10-10
%       * Initial release



%% Validate arguments
narginchk(1, 1);
nargoutchk(0, 1);

try
    validateattributes(th, {'numeric'}, {'nonempty', 'finite', 'nonnan', 'nonsparse'}, mfilename, 'th');
catch me
    throwAsCaller(me);
end



%% Calculate versine

v = 1 - cos(th);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

function h = haversin(z)
% HAVERSIN creates the haversinersine function of argument z
%
%   Inputs:
%
%   Z                   NxM matrix of values to calculate haversinersine of
%
%   Outputs:
%
%   H                   NxM matrix of haversinersine values
%
%   See also:
%   VERSIN



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
    validateattributes(z, {'numeric'}, {'nonempty', 'finite', 'nonnan', 'nonsparse'}, mfilename, 'z');
catch me
    throwAsCaller(me);
end



%% Calculate haversinersine

h = 1/2.*versin(z);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

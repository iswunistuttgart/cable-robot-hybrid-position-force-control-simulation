function w = f2w(f)
% F2W turns ordinary frequency into angular frequency
%
%   W = F2W(F) turns ordinary frequency F in hertz into angular frequency W in
%   rad/s.
%
%   Inputs
%
%   F: Ordinary frequency measured in hertz.
%
%   Outputs
%
%   W: Angular frequency measured in radians per second.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-06-27
% Changelog:
%   2016-06-27
%       * Initial release



%% Magic

w = 2*pi.*f;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

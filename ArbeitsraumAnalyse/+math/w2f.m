function f = w2f(w)
% W2F turns angular frequency into ordinary frequency
%
%   F = W2F(W) turns angular frequency W in rad/s into ordinary frequency F in
%   hertz.
%
%   Inputs
%
%   W: Angular frequency measured in radians per second.
%
%   Outputs
%
%   F: Ordinary frequency measured in hertz.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-06-27
% Changelog:
%   2016-06-27
%       * Initial release



%% Magic

f = w./(2*pi);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

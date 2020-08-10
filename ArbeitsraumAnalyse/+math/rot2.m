function R = rot2(a)
% ROT2 creates the 2D rotation matrix of angles A
%
%   Inputs:
%
%   A                   Nx1 vector of angles in radian to turn into rotation
%                       matrices.
%
%   Outputs:
%
%   R                   2x2xN rotation matrix for each agle.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-03-06
% Changelog:
%   2017-03-06
%       * Fix use of degree to radian
%   2017-03-05
%       * Add support for multiple angles in A. Now returns a 2x2xN matrix for
%       Nx1 vector A
%   2017-01-03
%       * Initial release



%% Do your code magic here
% Pre-calculate the sine and cosine of the arguments
st = sin(a);
ct = cos(a);

% Initialize matrix
R = zeros(2, 2, numel(a), 'like', a);

R(1,1,:) = ct;
R(1,2,:) = -st;
R(2,1,:) = st;
R(2,2,:) = ct;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

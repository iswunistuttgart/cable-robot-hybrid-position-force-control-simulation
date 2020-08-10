function R = rot2d(a)
% ROT2d creates the 2D rotation matrix of angles A given in degree
%
%   Inputs:
%
%   A                   Nx1 vector of angles in degree to turn into rotation
%                       matrices.
%
%   Outputs:
%
%   R                   2x2xN rotation matrix for each agle.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2018-04-24
% Changelog:
%   2018-04-24
%       * Initial release from `rot2`



%% Do your code magic here
% Pre-calculate the sine and cosine of the arguments
st = sind(a);
ct = cosd(a);

% Initialize matrix
R = zeros(2, 2, numel(a), 'like', a);

R(1,1,:) =  ct;
R(1,2,:) = -st;
R(2,1,:) =  st;
R(2,2,:) =  ct;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

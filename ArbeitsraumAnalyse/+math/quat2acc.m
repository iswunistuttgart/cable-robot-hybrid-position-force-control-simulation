function alpha = quat2acc(q_dot, q_ddot)%#codegen
% QUAT2ACC Get angular acceleration from quaternion velocity, and acceleration
% 
%   ALPHA = QUAT2ACC(DQ, DDQ) gets the angular acceleration ALPHA from
%   quaternion velocity DQ, and quaternion acceleration DDQ.
%
%   Inputs:
% 
%   DQ              Nx4 vector of quaternion velocity.
%
%   DDQ             Nx4 vector of quaternion acceleration.
% 
%   Outputs:
% 
%   ALPHA           Nx3 vector of angular accelerations.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2017-08-05
% Changelog:
%   2017-08-05
%       * Add input validation
%       * Backwards incompatible change: allow for multiple quaternion rate
%       changes to be converted to angular velocities.
%   2016-12-09
%       * Propagate rename of function `ascolumn` to `ascol`
%   2016-05-10
%       * Add help doc
%   2016-04-08
%       * Initial release



%% Validate input
% Number of quaternions to manipulate
nQuaternions = size(q, 1);
% Quaternions must be given as Nx4 matrix
validateattributes(q, {'numeric'}, {'2d', 'nonempty', 'ncols', 4, 'finite', 'nonsparse', 'nonnan'}, mfilename, 'q');
% Quaternion velocities must be given as Nx4 matrix, too
validateattributes(q, {'numeric'}, {'2d', 'nonempty', 'ncols', 4, 'nrows', nQuaternions, 'finite', 'nonsparse', 'nonnan'}, mfilename, 'q');



%% Calculation

alpha = 2*sum(bsxfun(@times, permute(quat2ratem(q_dot), [3, 1, 2]), permute(q_ddot, [1, 3, 2])), 3);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

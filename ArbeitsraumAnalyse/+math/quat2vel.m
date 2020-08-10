function omega = quat2vel(q, q_dot)%#codegen
% QUAT2VEL Converts quaternion velocity vector to angular velocity vector
% 
%   OMEGA = QUAT2VEL(Q, DQ) calculates the angular velocity vector OMEGA from
%   the quaternion position and velocity vectors Q and DQ, respectively.
%
%   Inputs:
% 
%   Q               Nx4 vector of quaternion in vector notation with the real
%       entry at the first index.
%
%   DQ              Nx4 vector of quaternion velocity.
% 
%   Outputs:
% 
%   OMEGA           Nx3 vector of angular velocities.



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



%% Magic
% The formula is quite simple: 2*W(q)*Dq
% To perform really quick calculations, we will do some magic with permute and
% bsxfun
omega = 2*sum(bsxfun(@times, permute(quat2ratem([q(:,1), q(:,2), q(:,3), q(:,4)]), [3, 1, 2]), permute(q_dot, [1, 3, 2])), 3);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

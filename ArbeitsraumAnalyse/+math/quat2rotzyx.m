function R = quat2rotzyx(q)%#codegen
% QUAT2ROTZYX Convert quaternion to Tait-Bryan angle rotation matrix
%
%   R = QUAT2ROTZYX(Q) calculates the rotation matrix following Tait-Bryan angle
%   convention for a given quaternion Q. If input Q is an Mx4 matrix, R is a
%   three-dimensional matrix containing M rotation matrices along the third
%   dimension.
%
%   
%   Inputs:
%   
%   Q: Quaternion or an Mx4 matrix of quaternions
%
%   Outputs:
%
%   R: Matrix of 3x3xM rotation matrices following tait-bryan angles convention
%   (yaw-pitch-roll <=> ZYX).
%
%   See also: QUAT2ANGLE, ANGLE2DCM, PERMUTE
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-04-08
% Changelog:
%   2016-04-08
%       * Initial release



%% Magic
% Get tait-bryan angles from the quaternion
[yaw, pitch, roll] = quat2angle(q, 'ZYX');

% Feed the tait-byran angles into the direction cosine matrix (watch out, this
% does an intrinsic rotation, thus we will need to transpose the result)
R = permute(angle2dcm(yaw, pitch, roll, 'ZYX'), [2, 1, 3]);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

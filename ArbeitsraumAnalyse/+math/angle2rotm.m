function R = angle2rotm(yaw, pitch, roll)%#codegen
% ANGLE2ROTM Create Tait-Bryan rotation matrices from rotation angles
%
%   R = ANGLE2ROTM(YAW, PITCH, ROLL) calculates the rotation matrix R for the
%   angles YAW, PITCH, ROLL following Tait-Bryan angle convention i.e., YAW
%   about Z, PITCH about Y-axis, and ROLL about X-axis following the order ZYX
%   i.e., first rotation about global X-axis, then global Y-axis, then global
%   Z-axis.
%
%
%   Inputs:
%
%   YAW: Array of M third rotation angles
%
%   PITCH: Array of M second rotation angles
%
%   ROLL: Array of M first rotation angles
%
%   Outputs:
%
%   R: Matrix of 3x3xM rotation matrices
%
%   See also: ANGLE2DCM
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-04-29
% Changelog:
%   2016-04-29
%       * Change name to ```angle2rotm```
%   2016-04-08
%       * Initial release



%% Magic
R = permute(angle2dcm(yaw, pitch, roll, 'ZYX'), [2, 1, 3]);


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

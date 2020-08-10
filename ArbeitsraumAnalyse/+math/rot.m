function R = rot(angle, axis)
%% ROT Rotate with an angle about the given axis
%
%   R = ROT(ANGLE, AXIS) rotates about axis AXIS with angle ANGLE.
%
%   Inputs:
%
%   ANGLE               Nx1 array of angles to rotate.
%
%   AXIS                zxis to rotate about. supported axes are 'X', 'Y', and
%                       'Z'.
%
%   Outputs:
%
%   R                   3x3xN matrix of rotation matrices with a rotation of A
%                       about axis AXIS.



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2019-01-17
% Changelog:
%   2019-01-17
%       * Initial release



%% Validate arguments

% ROT(ANGLE, AXIS)
narginchk(2, 2);
% ROT(ANGLE, AXIS)
% R = ROT(ANGLE, AXIS)
nargoutchk(0, 1);

validateattributes(angle, {'numeric'}, {'nonempty', 'vector', 'nonnan', 'finite', 'nonsparse'}, mfilename, 'angle')

validatestring(lower(axis), {'x', 'y', 'z'}, mfilename, 'axis');



%% Determine rotation matrix

% Number of angles
nangle = numel(angle);

% Turn the vector into a 3d matrix
angle = reshape(angle, [1, 1, nangle]);

% Pre-calculate sine and cosines
sa = sin(angle);
ca = cos(angle);
z = zeros(1, 1, nangle);
o = ones(1, 1, nangle);

% Different rotation matrices
switch lower(axis)
  case 'x'
    R = cat(2 ...
            , o,  z,    z ...
            , z,  ca,  sa ...
            , z, -sa,  ca ...
        );
  case 'y'
    R = cat(2 ...
            ,  ca, z, -sa ...
            ,   z, o,   z ...
            ,  sa, z,  ca ...
        );
  case 'z'
    R = cat(2 ...
          ,  ca, sa, z ...
          , -sa, ca, z ...
          ,   z,  z, o ...
        );
end

% Turn our current 1x9xNA array into a 3x3xNA array
R = reshape(R, [3, 3, nangle]);

% Vanish singular values
singular = abs(R) < 10 * eps(class(angle)) & R ~= 0;
R(singular) = 0;


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original author as
% can be found in the header Your contribution towards improving this function
% will be acknowledged in the "Changes" section of the header

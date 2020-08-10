function Rx = rotxsym(alpha)%#codegen
% ROTXSYM Symbolic rotation matrix about the x-axis
%
%   ROTMAT = ROTXSYM(ALPHA) returns the rotation matrix, ROTMAT, that rotates a
%   point around the x-axis for an angle ALPHA (in degrees). The point is
%   specified in the form of [x;y;z], with the x, y, and z axes forming a
%   right-handed Cartesian coordinate system. With the x axis pointing towards
%   the observer, ALPHA is measured counter-clockwise in the y-z plane.
%
%   ROTMAT is a 3x3 matrix. The rotation of the point can be achieved by
%   left-multiplying ROTMAT with the point's coordinate vector [x;y;z].
%
%   See also: ROTX
%



%% File information
% Author: Philipp Tempel <philipp.tempel@isw.uni-stuttgart.de>
% Date: 2016-09-02
% Changelog:
%   2016-09-02
%       * Formatted source code
%   2016-04-05
%       * Initial release



%% Assertion
assert(isa(alpha, 'sym'), 'Input must be symbolic');



%% Magic
Rx = [...
     1,   0,              0; ...
     0,   cos(alpha),    -sin(alpha); ...
     0,   sin(alpha),     cos(alpha)];


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

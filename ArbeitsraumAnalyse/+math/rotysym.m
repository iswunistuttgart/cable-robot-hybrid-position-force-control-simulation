function Ry = rotysym(beta)%#codegen
% ROTYSYM Symbolic rotation matrix about the y-axis
%
%   ROTMAT = ROTYSYM(BETA) returns the rotation matrix, ROTMAT, that rotates a
%   point around the y-axis for an angle BETA (in degrees). The point is
%   specified in the form of [x;y;z], with the x, y, and z axes forming a
%   right-handed Cartesian coordinate system. With the x axis pointing towards
%   the observer, BETA is measured counter-clockwise in the z-x plane.
%
%   ROTMAT is a 3x3 matrix. The rotation of the point can be achieved by
%   left-multiplying ROTMAT with the point's coordinate vector [x;y;z].
%
%   See also: ROTY
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
assert(isa(beta, 'sym'), 'Input must be symbolic');



%% Magic
Ry = [...
     cos(beta),     0,   sin(beta); ...
     0,             1,   0; ...
    -sin(beta),     0,   cos(beta)];


end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

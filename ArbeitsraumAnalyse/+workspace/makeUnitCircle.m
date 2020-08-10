function CircleVertices = makeUnitCircle(NumberOfCirclePoints)
%MAKEOCTAHEDRON Summary of this function goes here
%   Detailed explanation goes here

%% File information
% Author: Thomas Reichenbach <thomas.reichenbach@isw.uni-stuttgart.de>
% Date: 2019-06-25
% Changelog:
%   2019-06-25
%       * Initial release

%% Argument processing
delta = NumberOfCirclePoints;


%% Main Code
% Init angle vector
phi = linspace(0,360,delta);

% Get coordinates of unit circle
x = cosd(phi);
z = sind(phi);

% Make matrix
aVertices = [x; z];

%% Assign output quantities
% OctahedronVertices = 0;
CircleVertices = aVertices;

end
%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header

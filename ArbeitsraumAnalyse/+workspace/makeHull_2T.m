function HullBorder = makeHull(InitalPose, LowerBound, UpperBound, HullPrecision, NumberOfCirclePoints,...             
                     FrameAnchors, PlatformAnchors, Wrench, Fmin, Fmax) %#codegen

%% File information
% Author: Thomas Reichenbach <thomas.reichenbach@isw.uni-stuttgart.de>
% Date: 2019-06-25
% Changelog:
%   2019-06-25
%       * Initial release

%% Parse Variables
% Number of cables per platform
nCables = size(FrameAnchors, 2);

% Number of degrees of freedom
nDof = 2;


%% Argument processing
aPose = InitalPose;
lb = LowerBound;
ub = UpperBound;
epsilon = HullPrecision;
delta = NumberOfCirclePoints;
aPlatform = PlatformAnchors;
aFrame = FrameAnchors;
wp = Wrench;
fmin = Fmin;
fmax = Fmax;


%% Setup hull search directions (circle)
% Init unit octahedron
aSearchDirections = workspace.makeUnitCircle(delta);
nCircle = size(aSearchDirections,2);

%% Loop over LineSearch
polyVertices = zeros(2,nCircle);
for iS=1:nCircle
    v = aSearchDirections(:,iS);
    r = workspace.lineSearch_2T(aPose, lb, ub, epsilon, v, aFrame, aPlatform, wp, fmin, fmax);
    polyVertices(:,iS) = r;
end

%% Assign output quantities
HullBorder = polyVertices;

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
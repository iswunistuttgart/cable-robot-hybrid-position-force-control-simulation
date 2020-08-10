function [out] = m1R2T(Pose, FrameAnchors, PlatformAnchors) %#codegen

%% File information
% Author: Thomas Reichenbach <thomas.reichenbach@isw.uni-stuttgart.de>
% Date: 2019-03-06
% Changelog:
%   2019-03-06
%       * Initial release

%% Argument processing
% Three input arguments
narginchk(3, 3);
% Zero to three output arguments
nargoutchk(0, 0);

%% Parse Variables
% Get number of wires
nCables = size(PlatformAnchors, 2);

%% Assert variables
aPosition = Pose(1:2)';
aRotation = math.rotrow2m(Pose(3:6));
aFrame = FrameAnchors;
aPlatform = PlatformAnchors;

%% Main code
% Platforms w.r.t. world coordinate system
aPlatform0 = (repmat(aPosition, 1, nCables) + aRotation*aPlatform);

%% Plot
for iC=1:nCables
    plot([aFrame(1,iC),aPlatform0(1,iC)],[aFrame(2,iC),aPlatform0(2,iC)],'r');
end

%% Assign output quantities
% First output: structure matrix; required
out = 1;

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
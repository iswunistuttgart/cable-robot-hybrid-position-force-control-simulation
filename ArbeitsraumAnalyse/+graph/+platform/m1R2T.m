function [out] = m1R2T(Pose, PlatformAnchors) %#codegen

%% File information
% Author: Thomas Reichenbach <thomas.reichenbach@isw.uni-stuttgart.de>
% Date: 2019-03-06
% Changelog:
%   2019-03-06
%       * Initial release

%% Argument processing
% Three input arguments
narginchk(2, 2);
% Zero output arguments
nargoutchk(0, 0);

%% Parse Variables
% Get number of wires
nCables = size(PlatformAnchors, 2);

%% Assert variables
aPosition = Pose(1:2)';
aRotation = math.rotrow2m(Pose(3:6));
aPlatform = PlatformAnchors;

%% Main code
% Platforms w.r.t. world coordinate system
aPlatform0 = (repmat(aPosition, 1, nCables) + aRotation*aPlatform);

%% Plot
% Platform vertices
plot(aPlatform0(1,:),aPlatform0(2,:),'ko','MarkerSize', 3);
% Platform frame
line([aPlatform0(1,:),aPlatform0(1,1)],[aPlatform0(2,:),aPlatform0(2,1)],'Color','k');

%% Assign output quantities
% First output: structure matrix; required
out = 1;

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
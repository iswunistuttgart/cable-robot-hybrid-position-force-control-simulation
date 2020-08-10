function [out] = m1R2T(FrameAnchors) %#codegen

%% File information
% Author: Thomas Reichenbach <thomas.reichenbach@isw.uni-stuttgart.de>
% Date: 2019-03-06
% Changelog:
%   2019-03-06
%       * Initial release

%% Argument processing
% Three input arguments
narginchk(1, 1);
% Zero to three output arguments
nargoutchk(0, 3);

%% Parse Variables
% Get number of wires
nCables = size(FrameAnchors, 2);

%% Assert variables
aFrame = FrameAnchors;

%% Main code
line([aFrame(1,:),aFrame(1,1)],[aFrame(2,:),aFrame(2,1)],'Color','k','LineStyle','--');

%% Assign output quantities
% First output: structure matrix; required
out = 1;

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
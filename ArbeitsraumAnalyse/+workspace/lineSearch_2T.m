function HullVertex = lineSearch_2T(InitialPose, LowerBound, UpperBound, HullPrecision,...
                    SearchDirection, FrameAnchors, PlatformAnchors,...
                    Wrench, Fmin, Fmax) %#codegen

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
aPose = InitialPose;
m = InitialPose(1:2)';
%R_matrix = math.rotrow2m(InitialPose(3:6));
%R_vector = InitialPose(3:6);
lb = LowerBound;
ub = UpperBound;
epsilon = HullPrecision;
v = SearchDirection;
aPlatform = PlatformAnchors;
aFrame = FrameAnchors;
wp = Wrench;
fmin = Fmin;
fmax = Fmax;

%% Setup unlimited rotation discretization
% Normalize search direction
v0 = v;
v0 = v./norm(v0);

%% Main Code
warning off;

% Check if abortion criteria is true
if ub-lb < epsilon
    lambda = ub;
    r = m + lambda.*v0; %stop line search
% Else call function recursively
else
    lambda = 0.5.*(ub + lb);
    r = m + lambda.*v0;
    pose_new = r';
    [~, ui] = kinematics.ik.standard(pose_new, FrameAnchors, PlatformAnchors, '2T');
    [AT, ~] = kinetostatics.structm.m2T(FrameAnchors, ui);
    f = kinetostatics.fdist.closed_form(wp, AT, fmin, fmax);
    %f = kinetostatics.fdist.improved_closed_form(wp, AT, fmin, fmax);

    % Check if all cable forces within the given limits
    if all(f > fmin) > 0 && all(f < fmax) > 0
        lb = lambda;
    else
        ub = lambda;
    end
    
    % Call same function again
    r = workspace.lineSearch_2T(aPose, lb, ub, epsilon, v0, aFrame, aPlatform, wp, fmin, fmax);
end

%% Assign output quantities
HullVertex = r;
warning on;

end

%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
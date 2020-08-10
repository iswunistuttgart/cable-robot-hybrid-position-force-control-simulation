clear;
close all;

%% Robot Geometry
% Frame
% FrameAnchors = [ -0.306 , 0.306 ,  0.306 , -0.306 ;
%                   0.3 , 0.3 , -0.016 , -0.016 ];
 
load('Grenzen1R2T.mat');


FrameAnchors = [ ...
	[-4.70,  4.30] ...
	; [ 3.70,  3.20] ...
	; [ 3.50, -5.20] ...
	; [-4.50, -5.00] ...
	];

ai = FrameAnchors;
FrameAnchors = transpose(FrameAnchors);


%Platform
% PlatformAnchors = 20*[ -0.05 ,   0.05 ,  0.05 , -0.05 ;
%                  -0.01 , -0.01 , 0.01 , 0.01 ];
             
PlatformAnchors = 20*[ -0.02 ,   0.02 ,  0.02 , -0.02 ;
                        -0.01 ,   -0.01 ,  0.01 , 0.01 ];
             


%PlatformAnchors = transpose(PlatformAnchors);


%% Robot Properties
% Cable force limits
% fmin = 0.1;
% fmax = 50;
fmin = 50;
fmax = 1200;

%% Workspace computation settings
% Robot inital values
r = [0, 0];

angle = 0;

R = [cos(2*pi/380*angle), -sin(2*pi/360*angle); sin(2*pi/360*angle), cos(2*pi/360*angle)];
pose = [r, math.rotm2row(R)];

%wp = [0; -110; 0];

wp  = [0; -9.81*50;0];

% Workspace parameters
lb = 0;
ub = 1.2;
ub = 10;
epsilon = 0.001;
delta = 360;

% Compute force distribution of initial pose
[li, ui] = kinematics.ik.standard(pose, FrameAnchors, PlatformAnchors, '1R2T');
[AT, ~] = kinetostatics.structm.m1R2T(FrameAnchors, ui, R);
%fi = kinetostatics.fdist.improved_closed_form(wp, AT, fmin, fmax);
fi = kinetostatics.fdist.closed_form(wp, AT, fmin, fmax);

% Init rotation vector
R_vec = linspace(-5,5,20);
Phi = linspace(0,360,delta);
nPhi = size(Phi, 2);
nR = size(R_vec, 2);
R_mat = repmat(R_vec,nPhi,1);


% %% 3D Workspace
% % Compute workspaces with different rotations
% H1 = zeros(2,nPhi,nR);
% for iR=1:nR
%     R_init = math.rot2d(R_vec(iR));
%     pose_init = [r, math.rotm2row(R_init)];
%     H1(:,:,iR) = workspace.makeHull(pose_init, lb, ub, epsilon, delta, FrameAnchors, ...
%     PlatformAnchors, wp, fmin, fmax);
% end
% 
% % Plot in 3d
% fig1 = figure(1);
% hold on;
% grid on;
% axis([min(FrameAnchors(1,:))-0.1,max(FrameAnchors(1,:))+0.1, ...
%     min(FrameAnchors(2,:))-0.1,max(FrameAnchors(2,:))+0.1]);
% for iR = 1:nR
%     plot3(H1(1,:,iR),H1(2,:,iR),R_mat(:,iR))
% end
% % graph.platform.m1R2T(pose, PlatformAnchors);
% % graph.cables.m1R2T(pose, FrameAnchors, PlatformAnchors);
% hold off


%% 2D Workspace
% Compute workspace with initial rotation
H2 = workspace.makeHull(pose, lb, ub, epsilon, delta, FrameAnchors, ...
    PlatformAnchors, wp, fmin, fmax);

% plot in 2d
fig2 = figure(2);
axis([min(FrameAnchors(1,:))-0.1,max(FrameAnchors(1,:))+0.1, ...
   min(FrameAnchors(2,:))-0.1,max(FrameAnchors(2,:))+0.1]);
axis equal;
grid on;
hold on;
ws = fill(H2(1,:),H2(2,:),'c');
%graph.frame.m1R2T(FrameAnchors);
graph.platform.m1R2T(pose, PlatformAnchors);
graph.cables.m1R2T(pose, FrameAnchors, PlatformAnchors);
plot(ai(1,1), ai(1,2),'g*',ai(2,1), ai(2,2),'g*',ai(3,1), ai(3,2),'g*',ai(4,1), ai(4,2),'g*');
hold on
plot(border1(1),border1(2),'r+')
hold on
plot(border2(1),border2(2),'r+')
hold on
plot(border3(1),border3(2),'r+')
hold on
plot(border4(1),border4(2),'r+')
hold on
plot(border12(1),border12(2),'r+')
hold on
plot(border23(1),border23(2),'r+')
hold on
plot(border34(1),border34(2),'r+')
hold on
plot(border41(1),border41(2),'r+')
hold off
alpha(ws, 0.5)


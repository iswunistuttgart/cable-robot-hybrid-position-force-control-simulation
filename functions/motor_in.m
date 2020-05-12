function [TrajectoryProfile] = motor_in(Distance, MaxSpeed, MaxAcceleration, ...
    EndTime, TimePeriod)

%% File information
% Author: Jingshan Chen
% Date: 2019-05-28
% Changelog:
%   2019-05-28
%       * Initial release

%% Argument processing
xm = Distance;                   % [m] Seilhub
vm = MaxSpeed;                   % [m/s] Maximalseilgeschwindigkeit
am = MaxAcceleration;            % [m/s^2] Maximalseilbeschleunigung
tend = EndTime;                  % [s] Simulationszeit
t_takt = TimePeriod;             % Abtastzeitsfrequenz


%% Main Code
t1=(2 *vm)/am;
t2 = (-vm^3 + am^2* xm)/(am^2* vm);
t0 =0:t_takt:tend;
t0 = t0.';

%% Genrate 7-phase profile
% Index assignment
idx1 = all((t0 >= 0 & t0 < t1/2), 2);
idx2 = all((t0 >= t1/2 & t0 < t1), 2);
idx3 = all((t0 >= t1 & t0 < t1+t2), 2);
idx4 = all((t0 >= t1+t2 & t0 < (3/2*t1+t2)), 2);
idx5 = all((t0 >= 3/2*t1+t2  & t0 < t1*2+t2), 2);
idx6 = all((t0 >= 2*t1+t2),2);

% Trajactory
x1 = (am *t0.^3)./(3 *t1);
x2 = am.*t0.^2 - (am.*t0.^3)./(3*t1) - (am.*t0.*t1)./2 + (am*t1^2)/12;
x3 = 1/2*am.*(t0 - t1).*t1 + (am*t1^2)/4;
x4 = (am.*(3*t1^3 + 6 *t1^2* t2 - 6 *t1^2.*(-t0 + t1 + t2) +... 
          4.*(-t0 + t1 + t2).^3))./(12*t1);
x5 = (am.*(2.* t0.^3 - 13*t1^3 - 21*t1^2*t2 - 12*t1*t2^2 - 2*t2^3 + ...
      - 6.*t0.^2.*(2*t1+ t2) + 6.*t0.*(2*t1 + t2)^2))./(6*t1);
x6 = xm.*ones(size(t0,1),1);

% Collect trajectory
x1 = x1(idx1);
x2 = x2(idx2);
x3 = x3(idx3);
x4 = x4(idx4);
x5 = x5(idx5);
x6 = x6(idx6);

x = [x1; x2; x3; x4; x5; x6];

% Make timeseries
ts = timeseries(x,t0);

%% Assign output quantities
TrajectoryProfile = ts;


%------------- END OF CODE --------------
% Please send suggestions for improvement of this file to the original
% author as can be found in the header
% Your contribution towards improving this function will be acknowledged in
% the "Changes" section of the header
function [s_t,v_t,a_t,j_t, t, t_e] = sp_profile(dt, s_e, v_0, v_e, v_max, a_max, j_max)
% [s_t,v_t,a_t,j_t, t] = sp_profile(dt, s_e, v_a, v_e, v_max, a_max, j_max)
% calculates a 7 phase profile for distance s_e with given start and end
% velocities `v_a` and `v_e` as well as given maximum velocity, acceleration and
% jerk (`v_max`, `a_max`, `j_max`). The 7 phase profile means, that the returned
% trajectory is time-optimal under the given constraints. The returned values
% are sampled with time differences given by `dt`.
%
% Source: Müller, R. (1995). "Verbesserung des kinematischen und dynamischen 
%         Bewegungsverhaltens von Handhabungsgeräten mit geschlossenen 
%         kinematischen Teilketten". Diss., RTHW Aachen [Ch. 6]
%
% Input:
%   dt      sample time interval for the discretization of returned trajectory
%   s_e     total length of trajectory, generally calculated with curve integral
%           $ \int_0^{s_e} \sqrt{ \frac{dx}{ds}^2 + \frac{dy}{ds}^2 + \frac{dz}{ds}^2 } ds $
%   v_0     starting velocity [m/s]. Set to 0, when starting from rest
%   v_e     end velocity [m/s]. Set to 0, when ending in rest
%   v_max   maximum velocity (feed rate) [m/s]
%   a_max   maximum (de-)acceleration [m/s^2]
%   j_max   maximum jerk [m/s^3]
%
% Output:
%   s_t     s(t) distance vector $ \in \mathbb{R}^l$, where $l$ is defined by the
%           optimal end time and the time discretization passed in `dt`
%   v_t     v(t) optimal velocities [m/s] matching the `s_t` output $\in \mathbb{R}^l$
%   a_t     v(t) optimal accelerations [m/s^2] matching the `s_t` output $\in \mathbb{R}^l$
%   j_t     v(t) optimal jerk [m/s^3] matching the `s_t` output $\in \mathbb{R}^l$
%   t       time points vector for all trajectories
%   t_e     optimal end time (scalar)
%
%
% (c) 2017 ISW Universität Stuttgart, Christoph Hinze,
%          christoph.hinze@isw.uni-stuttgart.de
%
% example call:
% [s,v,a,j,t] = sp_profile(0.001, 10, 0, 0, 10, 21, 100);

% TODO: Treat special cases (v_max never reached, a_max never reached, s_e too short)
coder.extrinsic('warning');
coder.extrinsic('ismembertol')

validateattributes(dt, {'numeric'}, {'scalar', 'positive', 'finite'},'sp_profile');
validateattributes(s_e, {'numeric'}, {'scalar', 'positive', 'finite'},'sp_profile');
validateattributes(v_0, {'numeric'}, {'scalar', 'nonnegative', 'finite'},'sp_profile');
validateattributes(v_e, {'numeric'}, {'scalar', 'nonnegative', 'finite'},'sp_profile');
validateattributes(v_max, {'numeric'}, {'scalar', 'positive', 'finite'},'sp_profile');
validateattributes(a_max, {'numeric'}, {'scalar', 'positive', 'finite'},'sp_profile');
validateattributes(j_max, {'numeric'}, {'scalar', 'positive', 'finite'},'sp_profile');

delta_v_0 = v_max - v_0;
a_anf = sign(delta_v_0)*a_max;
j_anf = sign(delta_v_0)*j_max;

is_a_max_reached_0 = (abs(delta_v_0) >= a_anf^2/abs(j_anf));

if ~is_a_max_reached_0
   a_anf = sign(delta_v_0)*sqrt(delta_v_0*j_anf);
end

delta_v_e = v_e - v_max;
a_end = sign(delta_v_e)*a_max;
j_end = sign(delta_v_e)*j_max;

is_a_max_reached_e = (abs(delta_v_e) >= a_end^2/abs(j_end));

if ~is_a_max_reached_e
   a_end = sign(delta_v_e)*sqrt(delta_v_e*j_end);
end

% init time spans vector
delta_T = zeros(7,1);

% acceleration time spans
delta_T(1)  = (delta_v_0~=0)*a_anf/j_anf;
delta_T(2) = (delta_v_0~=0)*(delta_v_0/a_anf - a_anf/j_anf);
delta_T(3) = delta_T(1);

% de-acelleration time spans
delta_T(5) = (delta_v_e~=0)*a_end/j_end;
delta_T(6) = (delta_v_e~=0)*(delta_v_e/a_end - a_end/j_end);
delta_T(7) = delta_T(5);

% constant velocity phase
delta_s_anf = (v_0 + v_max)/2*sum(delta_T(1:3));
delta_s_end = (v_e + v_max)/2*sum(delta_T(5:7));

has_constant_vel_phase = (s_e>= delta_s_anf + delta_s_end);

if has_constant_vel_phase
    delta_T(4) = (s_e - delta_s_anf - delta_s_end)/v_max;
else
    % case, that v_max is not reached -> means t_4 = 0
    
    %delta_T(4) = 0 is already initialized
    % v_max is not reached -> calculate new v_max:
    u = a_anf/j_anf + a_end/j_end;
    v = 1/a_anf - 1/a_end;
    w = v_0*(a_anf/j_anf - v_0/a_anf) + v_e*(a_end/j_end - v_e/a_end) - 2*s_e;
    p_half = u/(2*v);
    
    v_max = -p_half + sqrt( p_half^2 - w/v);
    % recalculate with new v_max
    delta_v_0 = v_max - v_0;
    a_anf = sign(delta_v_0)*a_max;
    j_anf = sign(delta_v_0)*j_max;
    
    is_a_max_reached_0 = (abs(delta_v_0) >= a_anf^2/abs(j_anf));

    if ~is_a_max_reached_0
       warning('A case occured, where the maximum velocity and acceleration cannot be reached due to constraints. The desired distance is probably not reached.');
       a_anf = sign(delta_v_0)*sqrt(delta_v_0*j_anf);
    end
    
    delta_v_e = v_e - v_max;
    a_end = sign(delta_v_e)*a_max;
    j_end = sign(delta_v_e)*j_max;
    
    is_a_max_reached_e = (abs(delta_v_e) >= a_end^2/abs(j_end));

    if ~is_a_max_reached_e
        warning('A case occured, where the maximum velocity and acceleration cannot be reached due to constraints. The desired distance is probably not reached.');
        a_end = sign(delta_v_e)*sqrt(delta_v_e*j_end);
    end
    % acceleration time spans
    delta_T(1)  = (delta_v_0~=0)*a_anf/j_anf;
    delta_T(2) = (delta_v_0~=0)*(delta_v_0/a_anf - a_anf/j_anf);
    delta_T(3) = delta_T(1);
    
    % de-acelleration time spans
    delta_T(5) = (delta_v_e~=0)*a_end/j_end;
    delta_T(6) = (delta_v_e~=0)*(delta_v_e/a_end - a_end/j_end);
    delta_T(7) = delta_T(5);
    
end

% Time points, where a new phase starts
T = cumsum([0; delta_T]);

% time vector
t = 0:dt:T(end);

% time vector that gets 0, whenever a new time phase starts
tp = t - sum((T(2:end)*ones(1,length(t)) <= ones(length(delta_T),1)*t).*(delta_T*ones(1,length(t))) ,1);

% integrational constants vectors j_j, a_j, v_j and s_j with helper variables:
j_j = j_anf*( t<T(2) ) - j_anf*(T(3)<=t & t < T(4)) + j_end*(T(5)<=t & t < T(6)) - j_end*( t >= T(7) );
a_j = a_anf*(T(2)<=t & t<T(4)) + a_end*(T(6)<=t);

v_1 = v_0 + a_anf*delta_T(1)/2 ;
v_2 = v_1 + a_anf*delta_T(2);
% v_3 = v_4 := v_max
v_5 = v_max + a_end*delta_T(5)/2;
v_6 = v_5 + a_end*delta_T(6);

v_j = v_0*(t<T(2)) + v_1*( T(2)<=t & t<T(3) ) + v_2*( T(3)<=t & t<T(4) ) + ...
    v_max*(T(4)<=t & t< T(6)) + v_5*( T(6)<=t & t<T(7) ) + ...
    v_6*(T(7)<=t);

s_1_p =  v_0*delta_T(1) + j_anf*delta_T(1)^3/6;
s_2_p =  v_1*delta_T(2) + a_anf*delta_T(2)^2/2;
s_3_p =  v_2*delta_T(3) + a_anf*delta_T(3)^2/2 - j_anf*delta_T(3)^3/6;
s_4_p = v_max*delta_T(4);
s_5_p = v_max*delta_T(5) + j_end*delta_T(5)^3/6;
s_6_p = v_5*delta_T(6) + a_end*delta_T(6)^2/2 ;

s_j = s_1_p*(T(2)<=t) + s_2_p*(T(3)<=t) + s_3_p*(T(4)<=t) + s_4_p*(T(5)<=t) + ...
    s_5_p*(T(6)<=t) + s_6_p*(T(7)<=t) ;

% generate trajectories:
j_t = j_j;
a_t = j_j.*tp + a_j;
v_t = j_j.*(tp.^2)./2 + a_j.*tp + v_j;
s_t = j_j.*(tp.^3)./6 + a_j.*(tp.^2)./2 + v_j.*tp + s_j;


if ~ismembertol(s_t(end),s_e,1e-4) && (is_a_max_reached_0 && is_a_max_reached_e)
    warning('The generated trajectory s_t is shorter than desired. Maybe the sample time dt is too long');
end
    
if nargout>5
    t_e = T(end);
end

end


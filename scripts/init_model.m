clear;
close all;

%% Modellparameter
% Mechanik
D_Trommel = 0.1 ;                   % [m] Der Durchmesser der Trommel von Seilwinde
R_Trommel = D_Trommel/2 ;           % [m] Der Radius der Trommel von Seilwinde
i_Getriebe = 5;                     % [-] Getriebe?bersetzung
i_Seilwinde = 2*pi*R_Trommel;       % [1/m] Seilwinden?bersetzung
i_gesamt = i_Seilwinde/i_Getriebe;  % [m] Gesamt?bersetzung

J_Motor = 0.00075;               % [kg*m^2] Tr?gheitsmoment des Motors
J_Bremse = 0.000059;             % [kg*m^2] Tr?gheitsmoment der Bremse
J_Getriebe = 0.000153;           % [kg*m^2] Tr?gheitsmoment der Getriebe
J_Trommel = 0.008685;
J_Kupplung = 0.0010734;          % [kg*m^2] Tr?gheitsmoment der Kupplung
J_Lager = 0.000863796875;        % [kg*m^2] Tr?gheitsmoment der Lager
J_Riemen = 0.00002018;           % [kg*m^2] Tr?gheitsmoment des Riemens
J_Adapter = 0.00011205;          % [kg*m^2] Tr?gheitsmoment der Adapter
J_Seilwinde = J_Trommel + J_Riemen + J_Kupplung + 2*J_Lager + J_Adapter;
J_Pruefwinde = J_Motor + J_Bremse + J_Seilwinde;
J_Trommelwelle = 0.003;          % [kg*m^2] Annahme
J_Lastwinde = J_Motor + J_Bremse + J_Getriebe + J_Seilwinde/(i_Getriebe^2) + J_Trommelwelle; % ohne J_Trommelwelle zu gering, sodass Reglerparameter mit Aufbau nicht ?bereinstimmen


% Elektronik
I_max_eff = 34.7;                % [A] Leistungsteil Maximaler effektivwert des Stroms
n_max = 4900/60;                 % [1/s] Maximalgeschwindigkeitsbegrenzung
                                 % Einheit aus IndraWorks [rpm] (Range 0 - 6100 aus IndraWorks)
K_s = 70.5*60/1000;              % [Vs] Spannungskonstante (70.5Vmin/1000)
K_m = 1.14;                      % [Nm/A_eff] Momentenkonstante (1.14 aus Datenblatt)
R = 1.55;                        % [Ohm] Wicklungswiderstand  
L = 6.7e-03;                     % [H] Wicklungsinduktivit?t  
T_el = L/R;                      % [s] elektrische Zeitkonstante
U_z = 400;                       % [V] Zwischenkreisspannung
f_pwm = 4000;                    % [Hz] PWM-Frequenz (einstellbar ??ber Parameter P-0-0001)     
T_tot = 1/f_pwm;                 % [s] Totzeit

% Encoder
quant_phi = 2*pi/(4096*128*(2^14)); % [rad] Quantisierungsintervall Motorencoder (Wertdiskretisierung)


%% Regelung
% Lageregler
K_v = 1000/60;      % [1/s] Proportionalverst?rkung des Lagereglers 
                    % Die Einheit aus IndraWorks Ds [1000/min] (Range 0 - 655.35 aus IndraWorks Ds)
% Drehzahlregler
K_p = 0.8*2*pi;     % [Nms/rad] Proportionalverst?rkung des Drehzahlreglers (Range 0 - 2147483.647 aus IndraWorks Ds)
T_n = 0.004;        % [s] Nachstellzeit des Drehzahlreglers (Range 0 - 6553.5 aus IndraWorks Ds)

% Stromregler
K_pi1 = 5;          % [V/A] Proportionalverst?rkung des Stromreglers       
T_ni1 = 0.0035;     % [s] Nachstellzeit des Stromreglers
K_pi2 = K_pi1;      % [V/A] Proportionalverst?rkung des zweiten Stromreglers      
T_ni2 = T_ni1;      % [s] Nachstellzeit des zweiten Stromreglers

% Nach Zirn
% K_pi1 = 5*R*((((T_el+T_tot)^2)/(4*T_el*T_tot))-1); % [V/A] Proportionalverst?rkung des ersten Stromreglers
% T_ni1 = T_el*4*((K_pi1/R)/((K_pi1/(R+1))^2)); % [s] Nachstellzeit des ersten Stromreglers

% Abtastzeiten     
T_x_control  = 500e-6;       % [s] Abtastzeit des Lagereglers (2 Auswahl:Basic 500e-6/Advanced 250e-6)
T_v_control  = 250e-6;       % [s] Abtastzeit des Geschwindigkeitsreglers (2 Auswahl:Basic 250e-6/Advanced 125e-6) 
T_i_control  = 125e-6;       % [s] Abtastzeit des Stromreglers (Nach Datenblatt ist T_i_control abh?ngig von f_pwm)

% Seilkraftregler
K_fc = 0.001;
T_fc = 4;

%% Reibung
%Culomb_reibung
%Viskose_reibung
d_visc = 0.7;
%Haftreibung
k_stat = 2.5;

%% Robotereigenschaften
FrameAnchors = [ ...
    [-4.70;  4.30] ...
  , [ 3.70;  3.20] ...
  , [ 3.50; -5.20] ...
  , [-4.50; -5.00] ...
];

% Initial position state
y0 = [0, 0];

% Initial velocity state
Dy0 = [0, 0];

% Initial pose
p0 = [y0(1), y0(2)];

% Platform linear inertia (mass) [ kg ]
PlatformWeight = 10;


%% Seil
NumberOfCables = 4;

% Initial geometric length
Cable_InitialLength = vecnorm(FrameAnchors-p0');
Cable_InitialLength = Cable_InitialLength.*(1 - 5e-3);

% Cable length offset
Cable_Offset = zeros(1, NumberOfCables);

% Cable elasticities [ N / m ]
% Cable_Elasticity = 1e5 .* ones(1, NumberOfCables);
k_elast = 172e3.* ones(1, NumberOfCables);  % [N/m] Steifigkeit   % Wert von Valentin Schmidts Diss

% Cable viscosities [ N s / m ]
% Cable_Viscosity = Cable_Elasticity.* 0.005;
d_damp = k_elast.*0.03;

Cable_ReferenceLength = 1 .* ones(1, NumberOfCables);

Elasticity = 5e5;

%% Sollwertgenerierung
% Final time of simulation
tend_sim = 20;

% Stepsize
stepsize = 5e-5;
% Time vector
tsp = tspan(0, tend_sim, 1e-3);

% Poses: for now just stick with the initial pose
p = repmat(p0, numel(tsp), 1);

% Move to diag. corner
p = [-sin(tsp)*0.4, sin(tsp)*0.4];
% p = [-sin(tsp-tend*0.01)*0.5, sin(tsp-tend*0.01)*0.5].*(tsp >= tend*0.08);

% Move left
% p = [linspace(0,0.1,size(tsp,1))'];

% Move up
% p = [zeros(size(tsp)), linspace(0,0.5,size(tsp,1))'];

% Create pose list as time series object
in_poselist = timeseries(p, tsp);

% Vector of refined time output values
toutput = in_poselist.Time;

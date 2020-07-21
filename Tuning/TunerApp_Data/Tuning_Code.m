%% Create system data with slTuner interface
TunedBlocks = {'cable_robot_2t_4cables/Cable Force Control/Force Controller1'};
AnalysisPoints = {'cable_robot_2t_4cables/Winch Systems/Demux2/1'; ...
                  'cable_robot_2t_4cables/Winch Systems/Demux1/1'; ...
                  'cable_robot_2t_4cables/Cable Force Control/Demux3/1'; ...
                  'cable_robot_2t_4cables/Cable Force Control/Demux2/1'; ...
                  'cable_robot_2t_4cables/Cable Force Control/Demux1/1'};
% Specify the custom options
Options = slTunerOptions('RateConversionMethod','tustin',...
                 'AreParamsTunable',false);
% Create the slTuner object
st = slTuner('cable_robot_2t_4cables',TunedBlocks,AnalysisPoints,Options);

% Set the parameterization of the tuned block
cable_robot_2t_4cables_Cable_Force_Control_Force_Controller1 = tunablePID('cable_robot_2t_4cables_Cable_Force_Control_Force_Controller1','pi');
cable_robot_2t_4cables_Cable_Force_Control_Force_Controller1.Kp.Value = K_fc;
cable_robot_2t_4cables_Cable_Force_Control_Force_Controller1.Ki.Value = T_fc;
setBlockParam(CL0,'cable_robot_2t_4cables/Cable Force Control/Force Controller1',cable_robot_2t_4cables_Cable_Force_Control_Force_Controller1);

%% Create tuning goal to shape how the closed-loop system responds to a specific input signal
% Inputs and outputs
Inputs = {'cable_robot_2t_4cables/Cable Force Control/Demux2/1[fc_soll_1]'};
Outputs = {'cable_robot_2t_4cables/Winch Systems/Demux1/1[fc_ist]'};
% Tuning goal specifications
Tau = 0.02; % Time constant
% Create tuning goal for step tracking
StepTrackingGoal1 = TuningGoal.StepTracking(Inputs,Outputs,Tau);
StepTrackingGoal1.Name = 'StepTrackingGoal1'; % Tuning goal name

%% Create tuning goal to limit the sensitivity of feedback loops to disturbances
Locations = {'cable_robot_2t_4cables/Winch Systems/Demux1/1[fc_ist]'}; % Feedback loop locations
% Tuning goal specifications
MaxSensitivity = zpk(0,1000,1); % Maximum sensitivity as a function of frequency
% Create tuning goal for sensitivity
SensitivityGoal1 = TuningGoal.Sensitivity(Locations,MaxSensitivity);
SensitivityGoal1.Name = 'SensitivityGoal1'; % Tuning goal name

%% Create tuning goal to enforce specific gain and phase margins
Locations = {'cable_robot_2t_4cables/Winch Systems/Demux1/1[fc_ist]'}; % Feedback loop locations
% Tuning goal specifications
GainMargin = 7.6; % Required minimum gain margin
PhaseMargin = 45; % Required minimum phase margin
% Create tuning goal for margins
MarginsGoal1 = TuningGoal.Margins(Locations,GainMargin,PhaseMargin);
MarginsGoal1.Name = 'MarginsGoal1'; % Tuning goal name

%% Create option set for systune command
Options = systuneOptions();
Options.Display = 'off'; % Tuning display level ('final', 'sub', 'iter', 'off')

%% Set soft and hard goals
SoftGoals = [ StepTrackingGoal1 ; ...
              SensitivityGoal1 ; ...
              MarginsGoal1 ];
HardGoals = [];

%% Tune the parameters with soft and hard goals
[CL1,fSoft,gHard,Info] = systune(st,SoftGoals,HardGoals,Options)



%% View tuning results
% viewSpec([SoftGoals;HardGoals],CL1);

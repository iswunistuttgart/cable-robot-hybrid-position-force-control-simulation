%% Aufsetzen einer Tuner-Session + Übertragungsfunktion

mdl = 'cable_robot_2t_4cables';
%open_system(mdl);

Input = 'cable_robot_2t_4cables/Cable Force Control/Demux2/fc_soll_1';
Output = 'cable_robot_2t_4cables/Winch Systems/Demux1/fc_ist';

sslin = slLinearizer(mdl);

addPoint(sslin,{Input, Output});

sys = getIOTransfer(sslin, Input, Output);

P = bodeoptions;
P.FreqUnits = 'Hz';
P.XLim = [0.1, 10000];
P.Grid = 'on';
bodeplot(sys,P)

% PLOTS FROM SYSTEM-TUNER
mdl = 'cable_robot_2t_4cables';

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
ST_ForceControl_PI = slTuner(mdl,TunedBlocks,AnalysisPoints,Options);
% 
Inputs = {'cable_robot_2t_4cables/Cable Force Control/Demux2/1[fc_soll_1]'};
Outputs = {'cable_robot_2t_4cables/Winch Systems/Demux1/1[fc_ist]'};


% Transfer Function
T1 = getIOTransfer(ST_ForceControl_ClosedLoop,Inputs,Outputs);
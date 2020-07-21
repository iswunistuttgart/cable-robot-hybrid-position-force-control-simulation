%% Bode-Diagramm aus Transfer-Function plotten

opts = bodeoptions;
opts.FreqUnits = 'Hz';
opts.XLim = [0.1, 10000000];
opts.Grid = 'on';
opts.XLabel.String = 'Frequenz';
opts.YLabel.String = [{'Amplitude'} {'Phase'}];
%opts.YLabel.String(2) = 'Phase [°]';
opts.Title.String = 'Bodediagramm';
%opts.OutputVisible = [{'off'}];
%opts.InputVisible = [{'off'}];


figure('Name','Bodediagramm','Color','White')
bode(TF_FoceControl_ClosedLoop,opts);
hold on
bode(TF_FoceControl_ClosedLoop_PI,opts);
% ylabel('Seilkraft[N]');
% xlim([0 max(time)]);
legend('ohne Regler','mit Regler');

[Gm,Pm,Wcg,Wcp] = margin(TF_FoceControl_ClosedLoop)
[GmPI,PmPI,WcgPI,WcpPI] = margin(TF_FoceControl_ClosedLoop_PI)



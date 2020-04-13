% close all; clear;
warning('off','all');

%load("bl3_r08_5cm.mat");

odata = load ('Bl3d2_r12_4p5_7cm_Cal_LG.mat');
colmnumN = 1;
data.time = odata.data.time; % Assign time vector
%data.force = odata.data.tendonForceN(:,colmnumN); % Assigns forces
data.force = odata.data.force(:,colmnumN); % Assigns forces
data.act = RawEMGNormalizerYe(odata.data.emg(:,colmnumN)); % EMG2Act and assigns
%data.act = RawEMGNormalizerYe(odata.data.mVf_EMG(:,colmnumN)); % EMG2Act and assigns
data.length = odata.data.length(:,colmnumN) * 0.001;
%data.length = odata.data.muscleLength_mm(:,colmnumN) * 0.001;

% data.force = odata.data.tendonForceN(20000:20300,1);
% data.length = odata.data.muscleLength_mm(20000:20300,1)* 0.001;
% data.time = odata.data.time(20000:20300,1);
% data.act = RawEMGNormalizerYe(odata.data.mV_EMG(20000:20300,1));

params.T_act=34;
params.penation_angle=24/57.2958;
params.Po= 101.4;
params.Lo= 0.018;
params.M= 0.008;
  
params.kss= 1395.4;
params.kts= 999.92;
params.Cts_S= 0.3094;
params.Cts_L=0.039373;
params.Cce_S= 27.6;
params.Cce_L=7.6313;
params.act_factor= 0.21449;     % Time-delay factor


%constraints
constraints.positions.Thetap.lb = -360/57.2958;
constraints.positions.Thetap.ub = 360/57.2958;
constraints.positions.Xp.lb = -0.05;
constraints.positions.Xp.ub = 0.05;
constraints.positions.Xts.lb = -0.05;
constraints.positions.Xts.ub = 0.05;
constraints.positions.Xss.lb = -0.05;
constraints.positions.Xss.ub = 0.05;
constraints.positions.Xm.lb = -0.05;
constraints.positions.Xm.ub = 0.05;
constraints.positions.Xce.lb = -0.05;
constraints.positions.Xce.ub = 0.05;

% model call
stim = wfm9(data,params);

%is_constraint(stim,constraints,'display');

wfm_rsqr = Rsquared(stim.forces.Fm(1000:length(stim.forces.Fm))' ,data.force(1000:length(data.force))');
wfm_rmse = RMSE(stim.forces.Fm(1000:length(stim.forces.Fm))',data.force(1000:length(data.force))');

disp (['WFM R^2 ' num2str(wfm_rsqr)]);
disp (['WFM RMSE ' num2str(wfm_rmse)]);
disp (['WFM RMSE % Max Force ' num2str(wfm_rmse/max(data.force))]);

post_flight_visualizer(stim,data,params);
%save_all_plots('.');

figure
hold on
title('Measured and Predicted Force');
plot(data.time(1000:length(stim.forces.Fm)),data.force(1000:length(stim.forces.Fm)));
plot(data.time(1000:length(stim.forces.Fm)),stim.forces.Fm(1000:length(stim.forces.Fm)),'linestyle',':','color','r');
xlabel('Time s');
ylabel('Force N');
legend('measured','wfm9');
text(0,25, ['RMSE % Max Force ' num2str(num2str(wfm_rmse/max(data.force)))]);
text(0,35,['R^2 ' num2str(wfm_rsqr)]);
%saveas(1,'forces.eps');

%% Optimal Arm Trajectory:
clc;
close all;
clear all;

%% Set up:
iterLimit = 10000;
nNodes = 151; %51, 101
nlp = DirectCollocation(nNodes);
method = 'Explicit Euler';

%Time Variable:
t = nlp.addTime;
T(1) = t;

%Model Parameters:
m = 1; % body mass
c = 0.5; % damper
l1 = 1.225; % Length of Link 1
l2 = 1.225; % Length of Link 2
l3 = 0.875; % Length of Link 3

%Index Number:
iC = 1;
%% States
xm_j1 = nlp.addState(-l1*cos(pi/4),-10,10,'Description','State: xm', 'Length', nNodes);
dxm_j1 = nlp.addState(0,-Inf,Inf,'Description','State: dxm', 'Length', nNodes);
ym_j1 = nlp.addState(l1*sin(pi/4),0,10,'Description','State: ym', 'Length', nNodes);          
dym_j1 = nlp.addState(0,-Inf,Inf,'Description','State: dym', 'Length', nNodes);

xm_j2 = nlp.addState(l2*cos(pi/3)-l1*cos(pi/4),-10,10,'Description','State: xm', 'Length', nNodes);
dxm_j2 = nlp.addState(0,-Inf,Inf,'Description','State: dxm', 'Length', nNodes);
ym_j2 = nlp.addState(l2*sin(pi/3)+l1*sin(pi/4),0,10,'Description','State: ym', 'Length', nNodes);          
dym_j2 = nlp.addState(0,-Inf,Inf,'Description','State: dym', 'Length', nNodes);

xm_j3 = nlp.addState(l3*cos(pi/6)+l2*cos(pi/3)-l1*cos(pi/4),-10,10,'Description','State: xm', 'Length', nNodes);
dxm_j3 = nlp.addState(0,-Inf,Inf,'Description','State: dxm', 'Length', nNodes);
ym_j3 = nlp.addState(l3*sin(pi/6)+l2*sin(pi/3)+l1*sin(pi/4),0,10,'Description','State: ym', 'Length', nNodes);          
dym_j3 = nlp.addState(0,-Inf,Inf,'Description','State: dym', 'Length', nNodes);

%Control Inputs:
Fx_1 = nlp.addInput(0,-10,10,'Description','Input: Fx1', 'Length', nNodes);
Fy_1 = nlp.addInput(0,-10,10,'Description','Input: Fy1', 'Length', nNodes);

Fx_2 = nlp.addInput(0,-10,10,'Description','Input: Fx2', 'Length', nNodes);
Fy_2 = nlp.addInput(0,-10,10,'Description','Input: Fy2', 'Length', nNodes);

Fx_3 = nlp.addInput(0,-10,10,'Description','Input: Fx3', 'Length', nNodes);
Fy_3 = nlp.addInput(0,-10,10,'Description','Input: Fy3', 'Length', nNodes);

%Variables:
L1 = nlp.addVariable(0,-Inf,Inf,'Description','Variable: F1', 'Length', nNodes);
L2 = nlp.addVariable(0,-Inf,Inf,'Description','Variable: F1', 'Length', nNodes);
L3 = nlp.addVariable(0,-Inf,Inf,'Description','Variable: F1', 'Length', nNodes);

%Model Dynamics
nlp.addPdeConstraint(xm_j1, dxm_j1, T(iC), ...
    'Method', method, ...
    'Description', 'xm dynamics');
nlp.addPdeConstraint(dxm_j1, Fx_1./m-c.*dxm_j1, T(iC), ...
    'Method', method, ...
    'Description', 'dxm dynamics');
nlp.addPdeConstraint(ym_j1, dym_j1, T(iC), ...
    'Method', method, ...
    'Description', 'ym dynamics');
nlp.addPdeConstraint(dym_j1, Fy_1./m-c.*dym_j1, T(iC), ...
    'Method', method, ...
    'Description', 'dym dynamics');

nlp.addPdeConstraint(xm_j2, dxm_j2, T(iC), ...
    'Method', method, ...
    'Description', 'xm dynamics');
nlp.addPdeConstraint(dxm_j2, Fx_2./m-c.*dxm_j2, T(iC), ...
    'Method', method, ...
    'Description', 'dxm dynamics');
nlp.addPdeConstraint(ym_j2, dym_j2, T(iC), ...
    'Method', method, ...
    'Description', 'ym dynamics');
nlp.addPdeConstraint(dym_j2, Fy_2./m-c.*dym_j2, T(iC), ...
    'Method', method, ...
    'Description', 'dym dynamics');

nlp.addPdeConstraint(xm_j3, dxm_j3, T(iC), ...
    'Method', method, ...
    'Description', 'xm dynamics');
nlp.addPdeConstraint(dxm_j3, Fx_3./m-c.*dxm_j3, T(iC), ...
    'Method', method, ...
    'Description', 'dxm dynamics');
nlp.addPdeConstraint(ym_j3, dym_j3, T(iC), ...
    'Method', method, ...
    'Description', 'ym dynamics');
nlp.addPdeConstraint(dym_j3, Fy_3./m-c.*dym_j3, T(iC), ...
    'Method', method, ...
    'Description', 'dym dynamics');

%% Stance constraints:
%Initial Conditions: (Allowing it to choose anywhere in these regions to start)
nlp.addConstraint(-l1,xm_j1.initial,-l1/2);
nlp.addConstraint(-l2,xm_j2.initial,-l2/2);
nlp.addConstraint(-l3,xm_j3.initial,l3);

%Starts at rest:
nlp.addConstraint(0,dxm_j1.initial,0);
nlp.addConstraint(0,dym_j1.initial,0);
nlp.addConstraint(0,dxm_j2.initial,0);
nlp.addConstraint(0,dym_j2.initial,0);
nlp.addConstraint(0,dxm_j3.initial,0);
nlp.addConstraint(0,dym_j3.initial,0);

%Objectives:
nlp.addConstraint(0,dxm_j1.final,0);
nlp.addConstraint(0,dym_j1.final,0);
nlp.addConstraint(0,dxm_j2.final,0);
nlp.addConstraint(0,dym_j2.final,0);
nlp.addConstraint(0,dxm_j3.final,0);
nlp.addConstraint(0,dym_j3.final,0);

% End effector desired location:
nlp.addConstraint(1.5,xm_j3.final,1.5);
nlp.addConstraint(1.8,ym_j3.final,1.8);

% Desired End Time:
nlp.addConstraint(60,T(1).final,60);

% Prefered Solution Positions: (Restricts solution workspace)
nlp.addConstraint(0, ym_j2 - ym_j1, Inf);
nlp.addConstraint(0, ym_j2 - ym_j3, Inf);

%% Space Constraints
% nlp.addConstraint(l1.^2,(ym_j1-0).^2 + (xm_j1-0).^2,l1.^2); %length constraint between base and joint 1
% nlp.addConstraint(l2.^2,(ym_j2-ym_j1).^2 + (xm_j2-xm_j1).^2,l2.^2); %length constraint between joint 1 and joint 2
% nlp.addConstraint(l3.^2,(ym_j3-ym_j2).^2 + (xm_j3-xm_j2).^2,l3.^2); %length constraint between joint 2 and joint 3

%Slack Variable Method:
link_1 = (ym_j1-0).^2 + (xm_j1-0).^2;
link_2 = (ym_j2-ym_j1).^2 + (xm_j2-xm_j1).^2;
link_3 = (ym_j3-ym_j2).^2 + (xm_j3-xm_j2).^2;

nlp.addConstraint(0,L1.^2 - link_1,0);
nlp.addConstraint(0,L2.^2 - link_2,0);
nlp.addConstraint(0,L3.^2 - link_3,0);

nlp.addConstraint(l1,L1,l1); %length constraint between base and joint 1
nlp.addConstraint(l2,L2,l2); %length constraint between joint 1 and joint 2
nlp.addConstraint(l3,L3,l3); %length constraint between joint 2 and joint 3

%% Objectives:
% % No objective:
% nlp.addObjective(0, ...
%  	'Description','No objective')

%Force squared objective:
nlp.addObjective(trapz(Fx_3.^2 + Fy_3.^2,T(1))+trapz(Fx_2.^2 + Fy_2.^2,T(1))+trapz(Fx_1.^2 + Fy_1.^2,T(1)), ...
 	'Description','Force-squared objective');

tic
    optim = Ipopt(nlp);
    optim.options.ipopt.max_iter = iterLimit;
    optim.export;

    optim.solve;
toc

%% Solution:
xm1_sol = squeeze(eval(xm_j1));
dxm1_sol = squeeze(eval(dxm_j1));
ym1_sol = squeeze(eval(ym_j1));
dym1_sol = squeeze(eval(dym_j1));
xm2_sol = squeeze(eval(xm_j2));
dxm2_sol = squeeze(eval(dxm_j2));
ym2_sol = squeeze(eval(ym_j2));
dym2_sol = squeeze(eval(dym_j2));
xm3_sol = squeeze(eval(xm_j3));
dxm3_sol = squeeze(eval(dxm_j3));
ym3_sol = squeeze(eval(ym_j3));
dym3_sol = squeeze(eval(dym_j3));

Fx1_sol = squeeze(eval(Fx_1));
Fy1_sol = squeeze(eval(Fy_1));
Fx2_sol = squeeze(eval(Fx_2));
Fy2_sol = squeeze(eval(Fy_2));
Fx3_sol = squeeze(eval(Fx_3));
Fy3_sol = squeeze(eval(Fy_3));

phi1 = atan2d(ym1_sol,xm1_sol);
phi2 = atan2d(ym2_sol-ym1_sol,xm2_sol-xm1_sol);
phi3 = atan2d(ym3_sol-ym2_sol,xm3_sol-xm2_sol);

%Creates a .mat file to save the angles for the simulation
save('angles.mat','phi1','phi2','phi3');

%Cartesian Joint Trajectories:
figure(1)
plot(xm1_sol,ym1_sol, 'b')
hold on
plot(xm2_sol,ym2_sol, 'g')
hold on
plot(xm3_sol,ym3_sol, 'r')
hold on
plot(0,0, 'ko')
xlabel('X (m)')
ylabel('Y (m)')
xlim([-5 5]);
ylim([0 5]);
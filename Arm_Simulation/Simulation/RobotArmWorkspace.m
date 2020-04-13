%% Robot Arm Variable Workspace:
%Code by Jacob Hackett 01/18/2020
clc;clear all;close all;
load('..\TrajectoryPlanning\angles.mat')

%% Dimensions:
%World Variables:
tableX = 3;
tableY = 3;
tableZ = 0.1;
stopTime = 10;
sample = 100;
%% Robot Arm Model:
open_system('ArmModel.slx');

[robot,importInfo] = importrobot(gcs);
robot.DataFormat = 'row';

show(robot);

% Robot Variables:
gravity = -9.81;
mass1 = 1086.74;
mass2 = 148.06;
mass3 = 197.41;
mass4 = 53.16;
mass5 = 74.12;

config = homeConfiguration(robot);

% Force Due to Gravity
fext1 = externalForce(robot,'Body1',[0 0 0 0 0 gravity*mass1]);
fext2 = externalForce(robot,'Body2',[0 0 0 0 0 gravity*mass2],config);
fext3 = externalForce(robot,'Body3',[0 0 0 0 0 gravity*mass3],config);
fext4 = externalForce(robot,'Body4',[0 0 0 0 0 gravity*mass4],config);
fext5 = externalForce(robot,'Body5',[0 0 0 0 0 gravity*mass5],config);
fext = fext1+fext2+fext3+fext4+fext5;

%% Trajectory Calc:
% Joint Angle Math: (Corrections to format to axes)
phi1_corrected = (phi1(:)-90).';

phi2_corrected = (phi2.'-90)-phi1_corrected;

phi3_corrected = (phi3.'-90)-phi2_corrected;

%% Closes Initialization Model
%Comment the code below out if you would like to see the model:
close_system('ArmModel.slx');
close all;
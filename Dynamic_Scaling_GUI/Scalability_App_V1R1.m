clear all
close all
clc

% Hannah Rodgers
% Contact: hrodg15@gmail.com
% Team 516: Lunar Mobility Tool
% Spring 2020 

%{ 
    The goal of this is to dynamically scale the Lunar Mobility Tool. The 
    full-scale (scale = 1) will be intended for the mission to the moon.
    The smallest scale will be similar to the dimensions of the prototype. The
    goal is to create a function that can take user input for what scale is
    desired and then output dimensions (.csv). There are certain key dimensions
    that will be calcualted; the others will be dependent upon these. 

    Reference: "Dynamic Similarity and Scaling for Design of Dynamical Legged
    Robots"
    *Dynamic similarity involves geometry and kinematic similarity 

    As of January 24, 2020: Geometric scaling is complete for the lander
    and robot linkage lengths. 

    To Do:
        - Show the full scale model and the scaled model for comparison 
        - Make a GUI for scaling 
        - Create a 3D Model 
%} 

%{
    Key Dimensions: 
        1. Robot arm end-effector to payload handle
            - Linkage lengths
            - Scaling to a POINT in space 
        2. Vehicle bed height 
%}


%% Full Scale (Scale Factor = 1) Design

%lander parameters 
lander_height = 3; 
lander_width = 2; 

% Vehicle Structure 
vehicle.length = 1.75;
vehicle.wheelRadius = 0.4;
vehicle.height = 1.2-vehicle.wheelRadius;

% Creates the structure for the robotic arm parameter
arm.length = (lander_height+0.5);
arm.width = 0.25;
arm.base = arm.length*0.35;
arm.secondlink = arm.length*0.35;
arm.endeffector = arm.length*0.2;
% arm.gripper = arm.length*0.4;

% Volumes and Masses 
% density of titanium is 4.056 g/cm^3
% density of graphite epoxy is 2.26 g/cm^3
p = 1000; %4.056*(1/1000)*(100^3); % density 
g = 9.81; % m/s^2, lunar gravity 
% Volume*p = mass 
arm.mass1 = arm.base*pi*((arm.width/4)^2)*p;
arm.mass2 = arm.secondlink*pi*((arm.width/4)^2)*p;
arm.mass3 = arm.endeffector*pi*((arm.width/4)^2)*p;

% scaled mass from LeWan Soul =    2.1484e+03 kg 
% Mass * gravity = weight; 
arm.weight1 = arm.mass1*g
arm.weight2 = arm.mass2*g
arm.weight3 = arm.mass3*g

% arm.weight1 =  ((2.1484e+03)*0.2)*g
% % (((2.1484e+03)-((2.1484e+03)*0.2))/2)
% arm.weight2 = (((2.1484e+03)-((2.1484e+03)*0.2))/2)*g
% arm.weight3 = (((2.1484e+03)-((2.1484e+03)*0.2))/2)*g


% Base linkage of the arm position and orientation
arm.x_position = 1.75;
arm.y_position = vehicle.height+vehicle.wheelRadius; 
arm.angle = 128*pi/180;

% Second linkage of the arm position and orientation
arm.angle_L2 = 0*pi/180;
arm.x_position2 = arm.x_position+arm.base*cos(arm.angle);
arm.y_position2 = arm.y_position+arm.base*sin(arm.angle);

% End Effector linkage of the arm position and orientation
arm.angle_L3 = 200*pi/180;
arm.x_position3 = arm.x_position2+arm.secondlink*cos(arm.angle_L2);
arm.y_position3 = arm.y_position2+arm.secondlink*sin(arm.angle_L2);

% % Simplified Model of Gripper
% arm.angle_L4 = 270*pi/180;
% arm.x_position4 = arm.x_position3+arm.endeffector*sin(arm.angle_L3);
% arm.y_position4 = arm.y_position3+arm.endeffector*cos(arm.angle_L3)

% Prototype Arm 
LeWanxArm.base = 101.73/1000; % m 
LeWanxArm.secondlink = 101.73/1000; % m 
LeWanxArm.EE = 56.642/1000; % m 
% LeWanxArm.G = 114.808/1000; % m 

scale(1) = LeWanxArm.base/arm.base;
scale(2)=LeWanxArm.secondlink/arm.secondlink;
scale(3) = LeWanxArm.EE/arm.endeffector;
% scale(4) = LeWanxArm.G/arm.gripper;

scale_factor = linspace(round(mean(scale),2),1,10);



% v = VideoWriter('full_scale_for_simulation.avi'); % create video object
% v.Quality = 95; % sets the video quality 
% v.FrameRate = 10; 
% open(v); % open the video object 
F1 = scaling(scale_factor(1),vehicle,arm)
F2 = scaling(scale_factor(2),vehicle,arm);
F3 = scaling(scale_factor(3),vehicle,arm);
F4 = scaling(scale_factor(4),vehicle,arm);
F5 = scaling(scale_factor(5),vehicle,arm);
F6 = scaling(scale_factor(6),vehicle,arm);
F7 = scaling(scale_factor(7),vehicle,arm);
F8 = scaling(scale_factor(8),vehicle,arm);
F9 = scaling(scale_factor(9),vehicle,arm);
F10 = scaling(scale_factor(10),vehicle,arm);

% scale_factor = round(mean(scale),2);

% clf
% movie(F1,2)
% pause(1)
% clf
% movie(F2,2)
% 
% close(v)


[T1, T2, T3] = worstCaseTorque(arm,g) % kN*m

%% Scaling Function 

function [F] = scaling(scale_factor,vehicle,arm)
%lander parameters 
lander_height = 3*scale_factor;
lander_width = 2*scale_factor;

% Vehicle Structure 
vehicle.length = 1.75*scale_factor;
vehicle.wheelRadius = 0.4*scale_factor;
vehicle.height = (1.2*scale_factor)-vehicle.wheelRadius;

% Creates the structure for the robotic arm parameter
Sarm.length = arm.length*scale_factor;
Sarm.width = arm.width*scale_factor;
Sarm.base = arm.base*scale_factor;
Sarm.secondlink = arm.secondlink*scale_factor;
Sarm.endeffector = arm.endeffector*scale_factor;
% Sarm.gripper = arm.gripper*scale_factor;

% Base linkage of the arm position and orientation
Sarm.x_position = 1.75*scale_factor;
Sarm.y_position = (vehicle.height+vehicle.wheelRadius); 
Sarm.angle = 25*pi/180;

% Second linkage of the arm position and orientation
Sarm.angle_L2 = 0*pi/180;
Sarm.x_position2 = Sarm.x_position+Sarm.base*cos(Sarm.angle);
Sarm.y_position2 = Sarm.y_position+Sarm.base*sin(Sarm.angle);

% End Effector linkage of the arm position and orientation
Sarm.angle_L3 = 200*pi/180;
Sarm.x_position3 = Sarm.x_position2+Sarm.secondlink*cos(Sarm.angle_L2);
Sarm.y_position3 = Sarm.y_position2+Sarm.secondlink*sin(Sarm.angle_L2);

% % Simplified Model of Gripper
% Sarm.angle_L4 = 270*pi/180;
% Sarm.x_position4 = Sarm.x_position3+Sarm.endeffector*sin(Sarm.angle_L3);
% Sarm.y_position4 = Sarm.y_position3+Sarm.endeffector*cos(Sarm.angle_L3)

% Sets figure settings 
fig = figure(); %sets the location and size of drawable area for the new figure 
ax = axes('XLim',[-1 10],'YLim',[-1 10]);

% Parent handler of the transform
hgTx = hgtransform('Parent',ax);
hgTxB = hgtransform('Parent',ax);
hgTxS = hgtransform('Parent',ax);
hgTxE = hgtransform('Parent',ax);
% hgTxG = hgtransform('Parent',ax);

% rectangle is in the form [x y w h] 

% Creates the vehicle body object 
vehicle_body = rectangle('Parent', hgTx, 'Position',[(lander_width/2)+(0.25*scale_factor) vehicle.wheelRadius vehicle.length vehicle.height], 'FaceColor','red');

% Creates the lander body object 
lander_body = rectangle('Parent', hgTx, 'Position',[-lander_width/2 scale_factor lander_width lander_height-(1*scale_factor)], 'EdgeColor','black');
lander_legL = rectangle('Parent', hgTx, 'Position',[-lander_width/2 0 lander_width*0.1 scale_factor], 'FaceColor','black');
lander_legR = rectangle('Parent', hgTx, 'Position',[(lander_width/2)-lander_width*0.1 0 lander_width*0.1 scale_factor], 'FaceColor','black');
lander_legM = rectangle('Parent', hgTx, 'Position',[0-((lander_width*0.1)/2) 0 lander_width*0.1 scale_factor], 'FaceColor','black');

% Creates objects to simulate the arm
base = rectangle('Parent', hgTxB, 'Position',[0 0 Sarm.base Sarm.width], 'EdgeColor','red');
link2 = rectangle('Parent', hgTxS, 'Position',[0 0 Sarm.secondlink Sarm.width], 'EdgeColor','red');
link3 = rectangle('Parent', hgTxE, 'Position',[0 0 Sarm.endeffector Sarm.width], 'EdgeColor','red');
% gripper = rectangle('Parent', hgTxG, 'Position',[0 0 Sarm.gripper Sarm.width], 'FaceColor','yellow');

% Rotation of Robot Linkages
rotationB = linspace(128,108,10)*(pi/180);
rotationS = linspace(82,34,10)*(pi/180);
rotationEE = linspace(-9,-3,10)*(pi/180);

y = -1:0.01:10; % for the dashed line indicating center of the lander 
x = 0*ones(1,length(y)); % enough zeros for vectors to be equal 

x2 = -1:0.01:10; % for the dashed line indicating center of the lander 
y2 = 1.5*ones(1,length(y)); % enough zeros for vectors to be equal 

% Text strings to show the lengths on the plot 
base_length = ['Length of Base: ',num2str(Sarm.base), 'm'];
second_length = ['Length of Link2: ',num2str(Sarm.secondlink), 'm'];
endeff_length = ['Length of End Effector: ',num2str(Sarm.endeffector), 'm'];
title_str = ['Robotic Arm Simulation at Scale ',num2str(round(scale_factor,2))];
% gripper_length = ['Length of Gripper: ',num2str(arm.gripper), 'm'];

% v = VideoWriter('scaled1.mp4'); % create video object
% v.Quality = 50; % sets the video quality 
% open(v); % open the video object 

for i = 1:length(rotationS)
    Sarm.angle = rotationB(i); 
    Sarm.angle_L2 =  rotationS(i); % rotating the secondary linkage
    Sarm.angle_L3 = rotationEE(i); % rotating the end effector 
    
    Sarm.x_position2 = Sarm.x_position+Sarm.base*cos(Sarm.angle); % secondary linkage position updates based on base link
    Sarm.y_position2 = Sarm.y_position+Sarm.base*sin(Sarm.angle);

    Sarm.x_position3 = Sarm.x_position2+Sarm.secondlink*cos(Sarm.angle_L2); % EE updates position based on secondary linkage 
    Sarm.y_position3 = Sarm.y_position2+Sarm.secondlink*sin(Sarm.angle_L2);
    
%     Sarm.x_position4 = Sarm.x_position3+Sarm.endeffector*cos(Sarm.angle_L3);
%     Sarm.y_position4 = Sarm.y_position3+Sarm.endeffector*sin(Sarm.angle_L3);

    drawrobot(Sarm,hgTxB,hgTxS,hgTxE); %calls the function to transform the object location 
    
    % makes the plots pretty 
    hold on 
    xlabel('X (m)')
    ylabel('Y (m)')
    title(title_str,'FontSize',15)
    text(7,7.5,base_length); 
    text(7,7,second_length);
    text(7,6.5,endeff_length); 
    plot(x,y,'k--')
    plot(x2,y2,'k--')
    grid minor 
    hold off 
    % the pause is so the loop doesn't fly through the rotation array 
    pause(0.5)    
    F(i) = getframe(fig); % grabs each frame of plot to save to movie 
%     writeVideo(v,f(i)); % write each frame to the video I'm creating 
end 


end 

%**************** drawRobot function ***********************************
function [] = drawrobot(arm,hgTxB,hgTxS,hgTxE)
% This function takes the position input and sends the robot to that
% location/trajectory. 
hgTxB.Matrix = makehgtform('translate', [arm.x_position arm.y_position 0], 'zrotate', arm.angle);
hgTxS.Matrix = makehgtform('translate', [arm.x_position2 arm.y_position2 0], 'zrotate', arm.angle_L2);
hgTxE.Matrix = makehgtform('translate', [arm.x_position3 arm.y_position3 0], 'zrotate', arm.angle_L3);
% hgTxG.Matrix = makehgtform('translate', [arm.x_position4 arm.y_position4 0], 'zrotate', arm.angle_L4,'yrotate',270);
end 

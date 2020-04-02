% Mass on a spring/damper
%for ball2
%added a second mass (motor) which will be applying the force that was seen
%in ball 1. This will add 2 states and 2 Pde constraints


close all
clear all
clc


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Constants%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
mb=20; %mass of ball
hb=1; %height of "ball" this is to adjust for zb not being the COM 
mm=20; %mass of motor
k=250;%spring const
b=15;%damping 
g=9.81;%gravity
lo=5;%natural spring length
init_vel=-2;


%% %%%%%%%%%%%%%%%%%%%%%%%Start Coalesce%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pde_method='trapezoidal'; %pde method
nNodes=201;%number of nodes
nlp=DirectCollocation(nNodes);

t=nlp.addTime;
T(1)=t;
t.initialGuess=1;
% t.maxIterations=2000;


F=nlp.addInput(0, -Inf, Inf, 'Description', 'Input: force','Length',nNodes);




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%state%%%%%%%%%%%%%%%%%%%%%%%%%%%%
zb=nlp.addState(0,-Inf,Inf,'Description','State: zb','Length',nNodes);
dzb=nlp.addState(0,-Inf,Inf,'Description','State: dzb','Length',nNodes);

%states added in ball2
%these states represent the addition of the motor
zm=nlp.addState(0,-Inf,Inf,'Description','State: zm','Length',nNodes);
dzm=nlp.addState(0,-Inf,Inf,'Description','State: dzm','Length',nNodes);


%% %%%%%%%%%%%%%%%%%%%%%%%%PDE CONSTRAINT%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nlp.addPdeConstraint(zb,dzb,t,'Method',pde_method,...
    'Description','dzb dynamics');

nlp.addPdeConstraint(dzb,(-F+k*(lo-zb)+b*(0-dzb)-mb*g)/mb,t,...
    'Method',pde_method,'Description','ddzb dynamics');

%pde constraints added with the addition of the motor mass
nlp.addPdeConstraint(zm,dzm,t,'Method',pde_method,...
    'Description','dzm dynamics');

nlp.addPdeConstraint(dzm,(F-(mm*g))/mm,t,'Method',pde_method,...
    'Description','ddzm dynamics');

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%constraints%%%%%%%%%%%%%%%%%%%%
%need to adjust initial velocity to be initial velocity of COM
nlp.addConstraint(init_vel,((dzb.initial*mb)+(dzm.initial*mm))/...
    (mb+mm),init_vel);

%need to constrain the motor to stay within a range of the rod/ball
%since zb goes to the bottom of the ball/rod the lowest zm can go is zb
%The motor cannot go higher than zb + the height of the ball/rod
nlp.addConstraint(0,zm-zb,hb);


% need a variable to hold |Power| 
% Pabs=F* ((dzb*mb)+(dzm*mm))/(mb+mm);


% nlp.addConstraint(0,F,0)
% nlp.addConstraint(0,F,150)

%%%% Time Constraint%%%% 
% nlp.addConstraint(.251,t,Inf);
% nlp.addConstraint(5,t,5);


%initially zb is at natural length of spring
nlp.addConstraint(0,lo-zb.initial,0);


%setting final COM vel to be = -initial COM vel
% nlp.addConstraint(-init_vel,((dzb.final*mb)+(dzm.final*mm))/(mb+mm),-init_vel)

%need initial spring force to = 0; because at touchdown the spring will be
%at natural length
% nlp.addConstraint(0,k*(lo-zb.initial),0);


%%%%%takeoff condition, force at ground is 0%%%%%
nlp.addConstraint(0,k*(lo-zb.final)+b*(-dzb.final),0);
% nlp.addConstraint(0,k*(lo-zb.final),0);

%%%%% preventing zb and zm from going through ground%%%%%
% nlp.addConstraint(0,zb,Inf);
% nlp.addConstraint(0,zb.final,Inf);
% nlp.addConstraint(0,zm,Inf);

com_init = ((mb*zb.initial)+(mm*zm.initial))/(mm+mb);
com_final = ((mb*zb.final)+(mm*zm.final))/(mm+mb);

dcom_init = ((mb*dzb.initial)+(mm*dzm.initial))/(mm+mb);
dcom_final = ((mb*dzb.final)+(mm*dzm.final))/(mm+mb);

nlp.addConstraint(0,-com_init + com_final + ((dcom_final^2)/g)...
    - ((dcom_init*dcom_final)/g) - (1/(2*g))*((dcom_final^2)...
    + dcom_init - 2*dcom_init*dcom_final),0)

%%%%%%%%%% objectives%%%%%%%%%%%%%%
% nlp.addObjective(0, 'Description', 'Minimize Time');
nlp.addObjective(trapz(F^2,t),'Description', 'Minimize F');
% nlp.addObjective(Pabs,'Descprition','integral(abs(F(t)*V(t)))')


%% %%%%%%%%%%%%%%%%%%%%run optimizator%%%%%%%%%%%%%%%%%%%%%%%
optim = Ipopt(nlp);
optim.export;
optim.solve;


%% %%%%%%%%%%%%%%%%%%%%%%%%Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%
tsol=linspace(0,squeeze(eval(t)),nNodes);

zbsol=squeeze(eval(zb))';
dzbsol=squeeze(eval(dzb))';

zmsol=squeeze(eval(zm))';
dzmsol=squeeze(eval(dzm))';

%solves for the COM position and velocity
comsol=(((zbsol+0.5*hb)*mb + (zmsol*mm))/(mm+mb))';
comsolvel= (((dzbsol*mb) + (dzmsol*mm))/(mm+mb))';

%solves for applied force
fsol=squeeze(eval(F))';

%solves for the spring force
FsSol= k*(lo-zbsol)';
FdSol=b*(-zbsol)';

zfsol=zeros(nNodes,1)';
zgsol= zfsol(1,:);
% keyboard

%%% testing to replace for loop
% Response
response = struct;
response.time = {tsol};
response.zm = zmsol;
response.zb = zbsol;
response.zf = zfsol;
response.zg = zgsol;

scene = ball2Scene(response);
Player(scene);    
     
%%  %%%%%%%%%%%%%%%%%%%%%%%%%%%%plots%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure(1) 
% plot(tsol,fsol)
% ylabel('F_applied')
% 
% figure(2)
% subplot(2,1,1)
% plot(tsol,zbsol)
% ylabel('zb')
% subplot(2,1,2)
% plot(tsol,dzbsol)
% ylabel('dzb')
% 
% figure(3)
% subplot(2,1,1)
% plot(tsol,zmsol)
% ylabel('zm')
% subplot(2,1,2)
% plot(tsol,dzmsol)
% ylabel('dzm')
% 
% figure(4)
% subplot(2,1,1)
% plot(tsol,comsol)
% ylabel('COM')
% subplot(2,1,2)
% plot(tsol,comsolvel)
% ylabel('dCOM')
% 
% figure(5)
% subplot(2,1,1)
% plot(tsol,FsSol)
% ylabel('F_s')
% subplot(2,1,2)
% plot(tsol,FdSol)
% ylabel('F_d')    
%     


% % keyboard

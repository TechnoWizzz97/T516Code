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
mf=1; %mass of foot
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

I_s=1;
I_f=2;
F(I_s)=nlp.addInput(0, -Inf, Inf, 'Description', 'Input: force','Length',nNodes);




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%state%%%%%%%%%%%%%%%%%%%%%%%%%%%%
zb(I_s)=nlp.addState(0,-Inf,Inf,'Description','State: zb(I_s)','Length',nNodes);
dzb(I_s)=nlp.addState(0,-Inf,Inf,'Description','State: dzb(I_s)','Length',nNodes);

%states added in ball2
%these states represent the addition of the motor
zm(I_s)=nlp.addState(0,-Inf,Inf,'Description','State: zm(I_s)','Length',nNodes);
dzm(I_s)=nlp.addState(0,-Inf,Inf,'Description','State: dzm(I_s)','Length',nNodes);

%%%% foot
zf(I_s) = nlp.addState(0,-Inf,Inf,'Description',...
    'State: zf', 'Length', nNodes);

dzf(I_s) = nlp.addState(0,-Inf,Inf,'Description',...
    'State: dzf', 'Length', nNodes);
%% %%%%%%%%%%%%%%%%%%%%%%%%PDE CONSTRAINT%%%%%%%%%%%%%%%%%%%%%%%%%%%%
nlp.addPdeConstraint(zb(I_s),dzb(I_s),t,'Method',pde_method,...
    'Description','dzb(I_s) dynamics');

nlp.addPdeConstraint(dzb(I_s),(-F(I_s)+k*(lo-zb(I_s))+b*(0-dzb(I_s))-mb*g)/mb,t,...
    'Method',pde_method,'Description','ddzb(I_s) dynamics');

%pde constraints added with the addition of the motor mass
nlp.addPdeConstraint(zm(I_s),dzm(I_s),t,'Method',pde_method,...
    'Description','dzm(I_s) dynamics');

nlp.addPdeConstraint(dzm(I_s),(F(I_s)-(mm*g))/mm,t,'Method',pde_method,...
    'Description','ddzm(I_s) dynamics');

nlp.addPdeConstraint(zf(I_s),dzf(I_s),T(I_s),'Method',pde_method,...
    'Description','dzf dynamics');

nlp.addPdeConstraint(zf(I_s),(k*(zb(I_s)-zf(I_s))+b*(dzb(I_s)-dzf(I_s))+mf*g)/mf,T(I_s),'Method',...
    pde_method,'Description','dzf dynamics');


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%constraints%%%%%%%%%%%%%%%%%%%%
%need to adjust initial velocity to be initial velocity of COM
nlp.addConstraint(init_vel,((dzb(I_s).initial*mb)+(dzm(I_s).initial*mm))/...
    (mb+mm),init_vel);

%need to constrain the motor to stay within a range of the rod/ball
%since zb(I_s) goes to the bottom of the ball/rod the lowest zm can go is zb
%The motor cannot go higher than zb + the height of the ball/rod
nlp.addConstraint(0,zm(I_s)-zb(I_s),hb);

%%%% Time Constraint%%%% 
% nlp.addConstraint(.251,t,Inf);
% nlp.addConstraint(5,t,5);


%initially zb is at natural length of spring
nlp.addConstraint(0,lo-zb(I_s).initial,0);


%need initial spring force to = 0; because at touchdown the spring will be
%at natural length
% nlp.addConstraint(0,k*(lo-zb.initial),0);


%%%%%takeoff condition, force at ground is 0%%%%%
nlp.addConstraint(0,k*(lo-zb(I_s).final)+b*(-dzb(I_s).final),0);
% nlp.addConstraint(0,k*(lo-zb.final),0);


com_init = ((mb*zb(I_s).initial)+(mm*zm(I_s).initial))/(mm+mb);
com_final = ((mb*zb(I_s).final)+(mm*zm(I_s).final))/(mm+mb);

dcom_init = ((mb*dzb(I_s).initial)+(mm*dzm(I_s).initial))/(mm+mb);
dcom_final = ((mb*dzb(I_s).final)+(mm*dzm(I_s).final))/(mm+mb);

nlp.addConstraint(0,-com_init + com_final + ((dcom_final^2)/g)...
    - ((dcom_init*dcom_final)/g) - (1/(2*g))*((dcom_final^2)...
    + dcom_init - 2*dcom_init*dcom_final),0)

%%%%%%%%%% objectives%%%%%%%%%%%%%%
% nlp.addObjective(0, 'Description', 'Minimize Time');
nlp.addObjective(trapz(F(I_s)^2,t),'Description', 'Minimize F');
% nlp.addObjective(Pabs,'Descprition','integral(abs(F(t)*V(t)))')


%% %%%%%%%%%%%%%%%%%%%%run optimizator%%%%%%%%%%%%%%%%%%%%%%%
optim = Ipopt(nlp);
optim.export;
optim.solve;


%% %%%%%%%%%%%%%%%%%%%%%%%%Animation %%%%%%%%%%%%%%%%%%%%%%%%%%%
tsol=linspace(0,squeeze(eval(t)),nNodes);

zbsol=squeeze(eval(zb(I_s)))';
dzbsol=squeeze(eval(dzb(I_s)))';

zmsol=squeeze(eval(zm(I_s)))';
dzmsol=squeeze(eval(dzm(I_s)))';

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

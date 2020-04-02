close all
clear all
clc


% % % % sys = stance_test2_system;

mr=10;%mass of rod
mm=10;%mass of motor
k=1;%spring const
b=1;%damping const
g=9.81;%gravity
lo=.5; %natural length of spring

rod_height=1;%to help determine COM of rod
motor_height=1;%to help determine COM of motor
Wr=mr*g;
Wm=mm*g;

final_com_pos=3;
fM_init=2;
fM_fin=-2;

fM_up=2;
fM_lwr=-fM_up;

init_vel=-2;


pde_method='Explicit Euler';
nNodes=101;%number of nodes
nlp=DirectCollocation(nNodes);

t=nlp.addTime;
t.initialGuess=1


%add input
%this is the applied force resulting from the motor
Fm=nlp.addInput(0, -Inf, Inf, 'Description', 'Input: f motor','Length',nNodes);


%States
zr=nlp.addState(0,-Inf,Inf,'Description','State: zr','Length',nNodes);
dzr=nlp.addState(0,-Inf,Inf,'Description','State: dzr','Length',nNodes);

zm=nlp.addState(0,-Inf,Inf,'Description','State: zm','Length',nNodes);
dzm=nlp.addState(0,-Inf,Inf,'Description','State: dzm','Length',nNodes);


%pde constraints
nlp.addPdeConstraint(zr,dzr,t,'Method',pde_method,...
    'Description','dzr dynamics');

nlp.addPdeConstraint(dzr, ((k*(lo-zr)+(b*dzm)-Fm-Wr)/mr),...
    t,'Method',pde_method,'Description','ddzr dynamics');

nlp.addPdeConstraint(zm,dzm,t,'Method',pde_method,...
    'Description','dzm dynamics');

nlp.addPdeConstraint(dzm,(Fm-Wm)/mm,t,'Method',pde_method,...
    'Description','ddzm dynamics');

%% normal constraints
Fs_init=k*(lo-zr.initial)/mr;
Fs_fin=k*(lo-zr.final)/mr;
Fs=k*(lo-zr)/mr;

nlp.addConstraint(0,Fs_init,0);%at the instant of touchdown the spring 
%force is still 0
% nlp.addConstraint(0,Fs_fin,0); %at the instant of liftoff the spring 
% force is equal to 

nlp.addConstraint(0,Fm,0);

nlp.addConstraint(0.09,t,Inf)

%at touch down (initial position) zr height should equal natural spring
%length
nlp.addConstraint(lo,zr.initial,lo);

%at touch down (initial position) dzr velocity should be <= 0
nlp.addConstraint(-Inf,dzr.initial,0);


%Initial COM position = final COM position ; the instant at touchdown is
%equal to the instant before liftoff 
%this does not need the "/(mm+mr) " because both sides will have it 
nlp.addConstraint(0,((mm*zm.initial)+(mr*zr.initial))-...
    ((mm*zm.final)+(mr*zr.final)),0);

%Final COM position
% nlp.addConstraint(final_com_pos,((mm*zm.final)+(mr*zr.final))/(mm+mr),...
%     final_com_pos)

%%initial COM velocity = - Final COM velocity 

%final COM vel = - initial COM vel
nlp.addConstraint(-init_vel, (mm*dzm.final + mr*dzr.final)/(mm+mr) ...
    ,-init_vel);

%initial COM vel =
nlp.addConstraint(init_vel, (mm*dzm.initial + mr*dzr.initial)/(mm+mr)...
    ,init_vel);



%% objectives and plots
nlp.addObjective(t, 'Description', 'Minimize Time');
% nlp.addObjective(trapz(Fm^2,t),'Description', 'Minimize F')
% nlp.addObjective(Pabs,'Descprition','integral(abs(F(t)*V(t)))')

%% run optimizator
optim = Ipopt(nlp);
optim.export;
optim.solve;

% response = struct
% response.time={time};
% response.
% % % scene=stance_test2_Scene(sys, response);
% % % Player(scene);
%% obtain data from expressions
fsol=squeeze(eval(Fm));
tsol=linspace(0,squeeze(eval(t)),nNodes);
zrsol=squeeze(eval(zr));
dzrsol=squeeze(eval(dzr));
zmsol=squeeze(eval(zm));
dzmsol=squeeze(eval(dzm));

COM_pos=mm*zmsol+ mr*zrsol;
COM_vel=mm*dzmsol+ mr*dzrsol;


%fix this
% F_s=k*(lo-zrsol)+b*(-dzrsol);



%% plot
figure(1)
plot(tsol,fsol)
ylabel('F_m')

figure(2)
subplot(2,1,1)
plot(tsol,zrsol)
ylabel('zr')
subplot(2,1,2)
plot(tsol,dzrsol)
ylabel('dzr')

figure(3)
subplot(2,1,1)
plot(tsol,zmsol)
ylabel('zm')
subplot(2,1,2)
plot(tsol,dzmsol)
ylabel('dzm')

figure(4)
subplot(2,1,1)
plot(tsol,COM_pos)
ylabel('COM')
subplot(2,1,2)
plot(tsol,COM_vel)
ylabel('dCom')
% 
% figure(5)
% plot(tsol,F_s)
% ylabel('F_s')
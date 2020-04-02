close all
clear all
clc


m_r=10;% mass of rod
m_m=10;%motor mass
k=1;%spring coef
b=1;%damp coef
g=9.81;%gravity
lo=2;

f_m_init=2; %initial force from motor
f_m_fin=0;%final force from motor
f_m_up=2;%motor force upper limit
f_m_lwr=-2;%motor force lower limit

zr_init=0;%initial Center of gravity (height)of rod
zm_init=0;%initial Center of gravity (height)of motor

dzm_fin=0%final vel of motor (mount)
dzm_init=0;%initial vel of motor

dzr_init=0;
dzr_fin=0;

desiredZ_r=5;%desired end location of rod

ped_method='Explicit Euler';

nNodes=101; 
nlp = DirectCollocation(nNodes);

%%  add states
t=nlp.addTime;

%add states
zr=nlp.addState(0,-Inf,Inf,'Description','State: zr','Length',nNodes);
dzr=nlp.addState(0,-Inf,Inf,'Description','State: dzr','Length',nNodes);

zm=nlp.addState(0,-Inf,Inf,'Description','State: zm','Length',nNodes);
dzm=nlp.addState(0,-Inf,Inf,'Description','State: dzm','Length',nNodes);

zcom=nlp.addState(0,-Inf,Inf,'Description','State: Center of mass position','Length',nNodes);

% Pabs=nlp.addState(0,-Inf,Inf,'Description','State: abs(F*V)','Length',nNodes);

%new variables
%Spring forces
% Fs_init=-k*zm.initial;
% Fs=-k*zm;

%Leg
% Fleg_fin=0;
% L_leg_init=lo;

t.initialGuess=1
%% Add inputs
%this is the applied force resulting from the motor
f_m=nlp.addInput(0, -Inf, Inf, 'Description', 'Input: f','Length',nNodes);

%% add dynamics
nlp.addPdeConstraint(zr,dzr,t,'Method',ped_method,'Description','dzr dynamics');

nlp.addPdeConstraint(dzr,(-f_m/m_r) + (k*(lo-zr)/m_r) + (b*(-dzr)/m_r) -(g) ...
    ,t,'Method', ped_method, 'Description', 'ddzr dynamics')
% nlp.addPdeConstraint(dzr,(-f_m/m_r) + (k*(-zr)/m_r)-(g) ...
%     ,t,'Method', ped_method, 'Description', 'ddzr dynamics')



nlp.addPdeConstraint(zm,dzm,t,'Method',ped_method,'Description','dzm dynamics');
nlp.addPdeConstraint(dzm,(f_m/m_m)-g,t,'Method',ped_method,'Description','ddzm dynamics');


%% add standard constraints 

nlp.addConstraint(f_m_lwr,f_m,f_m_up); %Force from motor limiter

%initial constraints
% nlp.addConstraint(zr_init,zr.initial,zr_init);%initial zr
% nlp.addConstraint(zm_init,zm.initial,zm_init);%initial zm
% % nlp.addConstraint(dzm_init,dzm.initial,dzm_init); %sets initial vel 


% final contraints
% make final position = initial position
% nlp.addConstraint(zm.initial,zm.final,zm.initial);%initial zm
% nlp.addConstraint(zr_init,zr.final,zr.initial);%final zr
% % nlp.addConstraint(desiredZ_r,zr.final,desiredZ_r)

%% Center of mass constraints
% nlp.addConstraint(-20,dzr,20);
% nlp.addConstraint(-20,dzm,20);

%Force of spring
nlp.addConstraint(0,(k*(lo-zr.final)/m_r),0)

%initial COM vel is -1
% nlp.addConstraint(-1,m_r*dzr.initial +m_m*dzm.initial,-1);


%initial COM vel = -final COM vel
nlp.addConstraint(0, (m_m*dzm.initial + m_r*dzr.initial)...
    + (m_m*dzm.final + m_r*dzr.final) ,0);

%final COM velocity has to be = 1 
%makes the hopper lift off
nlp.addConstraint(1, m_m*dzm.final + m_r*dzr.final ,1);

%initial COM position = final COM position 
nlp.addConstraint(0,(m_m*zm.initial + m_r*zr.initial)...
    - (m_m*zm.final + m_r*zr.final),0)

%sets zr.final = natural spring length
nlp.addConstraint(0,lo-zr.final,0)

%dzr has to be negative at touch down. this is because the damper will
%instantly change because the instant change in velocity
%therefore the Force can't be 0. 
nlp.addConstraint(-Inf,dzr.final,0)

%time has to be greater than 0.1
% nlp.addConstraint(.01,t,Inf)
% % nlp.addConstraint(-dzm_init,dzm.final,-dzm_init);%sets final vel

% % Constraints for objective (abs(F(t)*V(t))
% nlp.addConstraint(f_m*dzr,Pabs,Inf); 
% nlp.addConstraint(-f_m*dzr,Pabs,Inf);

%% objectives and plots
nlp.addObjective(trapz(f_m^2,t),'Description', 'Minimize F')
% nlp.addOBjective(Pabs,'Descprition','integral(abs(F(t)*V(t)))')


optim = Ipopt(nlp);
optim.export;
optim.solve;

fsol=squeeze(eval(f_m));
tsol=linspace(0,squeeze(eval(t)),nNodes);
zrsol=squeeze(eval(zr));
dzrsol=squeeze(eval(dzr));
zmsol=squeeze(eval(zm));
dzmsol=squeeze(eval(dzm));

COM_pos=m_m*zmsol+ m_r*zrsol;
COM_vel=m_m*dzmsol+ m_r*dzrsol;

F_s=k*(lo-zrsol);

figure(1)
plot(tsol,fsol)
ylabel('F')

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

figure(5)
plot(tsol,F_s)
ylabel('F_s')
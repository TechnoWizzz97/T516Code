% Mass on a spring/damper

close all
clear all
clc

mb=10;
k=50;
b=15;
g=9.81;
lo=5;

pde_method='trapezoidal';
nNodes=201;%number of nodes
nlp=DirectCollocation(nNodes);

t=nlp.addTime;
t.initialGuess=5;


F=nlp.addInput(0, -Inf, Inf, 'Description', 'Input: force','Length',nNodes);

%state
zb=nlp.addState(0,-Inf,Inf,'Description','State: zb','Length',nNodes);
dzb=nlp.addState(0,-Inf,Inf,'Description','State: dzb','Length',nNodes);


%PDE CONSTRAINT
nlp.addPdeConstraint(zb,dzb,t,'Method',pde_method,...
    'Description','dzb dynamics');

nlp.addPdeConstraint(dzb,(F+k*(lo-zb)+b*(0-dzb)-mb*g)/mb,t,...
    'Method',pde_method,'Description','ddzb dynamics');
 
%constraints
% nlp.addConstraint(0,F,0)
% nlp.addConstraint(0,F,150)

nlp.addConstraint(0,t,5);

%initially zb is at natural length of spring
nlp.addConstraint(0,lo-zb.initial,0);
%zb is always positive (can't go into the ground)
% nlp.addConstraint(0,zb,Inf);

%final position is half the natural length of the spring
nlp.addConstraint(2*lo,zb.final,lo*2);

%initial velocity = 0
nlp.addConstraint(0,dzb.initial,0);

% nlp.addObjective(0, 'Description', 'Minimize Time');
nlp.addObjective(trapz(F^2,t),'Description', 'Minimize F');

%% run optimizator
optim = Ipopt(nlp);
optim.export;
optim.solve;

fsol=squeeze(eval(F));
tsol=linspace(0,squeeze(eval(t)),nNodes);
zbsol=squeeze(eval(zb));
dzbsol=squeeze(eval(dzb));


figure(1) 
plot(tsol,fsol)
ylabel('f')

figure(2)
subplot(2,1,1)
plot(tsol,zbsol)
ylabel('zb')
subplot(2,1,2)
plot(tsol,dzbsol)
ylabel('dzb')

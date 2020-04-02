clc;
clear all;
close all;

m = 80;                                        % [kg] mass of body
geom.Ls = 0.4;                            % [m] Length of Shank ( source: Sawicki )
geom.Lpf0 = 0.4;                        % [m] Initial Length of PF 
geom.Lpf_rest = 0.368;              % [m] Neutral Lenght of Plantarflexor ( source: Sawicki )
geom.Lma = 0.04;                      % [m] Length of moment arm ( heel-to-ankle link )
geom.Lf = 0.256;                        % [m] Length of foot ( ankle-to-toe link )
geom.phi0 = acos((geom.Lpf0^2+geom.Lma^2-geom.Ls^2)/(2*geom.Lpf0*geom.Lma));
geom.th0 = deg2rad(95);           % [r] Initial ankle angle
geom.alpha = deg2rad(160);      % [r] Angle of foot arch
geom.Lfp = sqrt(geom.Lma^2+geom.Lf^2-2*geom.Lma*geom.Lf*cos(geom.alpha));       % [m] Length of footprint
geom.psi = acos((geom.Lf^2+geom.Lfp^2-geom.Lma^2)/(2*geom.Lfp*geom.Lf));                % [r] Angle of toe-to-footprint
geom.ya0 = geom.Lf*sin(geom.psi);             % [m] Initial y-ankle pos

g = 9.81;                                     % [m/s2] Gravitational constant
Kpf = 315.4;                               % [N/mm] Linear PF stiffness ( source: Sawicki )
Bpf = 12.5;                                 % [Ns/m] PF dampening (arbitrary value)
I = 15;                                         % [N*m2] Moment of ankle unit ( true value not solved for )


Flim = 6000;                    % [N] Force production limit  ( source: Sawicki )
Vlim = 0.326;                   % [m/s] Contraction velocity limit ( source: Sawicki )

IntegrationMethod = 'Trapezoidal';

nNodes = 51;

nlp = DirectCollocation(nNodes); 

% Add Time
t = nlp.addTime;

%                                   [ Knee States, Ankle States, Theta]
%% Add States [ xk, dxk, yk, dyk, xa, dxa, ya, dya, th, dth]
xk = nlp.addState(0,-Inf, Inf, 'Description','State: Xk','Length', nNodes);
dxk = nlp.addState(0,-Inf, Inf, 'Description','State: dXk','Length', nNodes);
yk = nlp.addState(0,-Inf, Inf, 'Description','State: Yk','Length', nNodes);
dyk = nlp.addState(0,-Inf, Inf, 'Description','State: dYk','Length', nNodes);
xa = nlp.addState(0,-Inf, Inf, 'Description','State: Xa','Length', nNodes);
dxa = nlp.addState(0,-Inf, Inf, 'Description','State: dXa','Length', nNodes);
ya = nlp.addState(0,-Inf, Inf, 'Description','State: Ya','Length', nNodes);
dya = nlp.addState(0,-Inf, Inf, 'Description','State: dYa','Length', nNodes);
ddxk = nlp.addState(0, -Inf, Inf, 'Description','State: ddXk','Length', nNodes);
ddyk = nlp.addState(0, -Inf, Inf, 'Description','State: ddYk','Length', nNodes);
th = nlp.addState(0,-Inf, Inf, 'Description','State: Theta','Length', nNodes);
thdot = nlp.addState(0,-Inf, Inf, 'Description','State: Theta dot','Length', nNodes);
%Lpf = nlp.addState(Lpf0, -Inf, Inf, 'Description','State: PF Length', 'Length', nNodes);

%% Add Inputs
F = nlp.addInput(0, -Inf, Inf, 'Description', 'Input: PF Force','Length', nNodes);

%% Add Variables
Lpf = nlp.addVariable(1, -Inf, Inf, 'Description','Variable: PF Length','Length', nNodes);
phi = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: PF Moment Angle', 'Length', nNodes);
delth = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: Diff Theta', 'Length', nNodes);
beta = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: PF-Shank Angle', 'Length', nNodes);
gamma = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: Grav-Shank Angle', 'Length', nNodes);
GRFx = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: Ground Reaction Force - Y','Length', nNodes);
GRFy = nlp.addVariable(0, -Inf, Inf, 'Description', 'Variable: Ground Reaction Force - X','Length', nNodes);

%% Add dynamics
nlp.addPdeConstraint(xk,dxk,t, 'Method', IntegrationMethod,'Description','xkdot dynamics');
nlp.addPdeConstraint(yk, dyk, t, 'Method', IntegrationMethod,'Description','ykdot dynamics');
nlp.addPdeConstraint(xa, dxa, t, 'Method', IntegrationMethod,'Description','xadot dynamics');
nlp.addPdeConstraint(ya, dya, t, 'Method', IntegrationMethod,'Description','yadot dynamics');
nlp.addPdeConstraint(dxk, ddxk, t, 'Method', IntegrationMethod,'Description','xkddot dynamics');
nlp.addPdeConstraint(dyk, ddyk, t, 'Method', IntegrationMethod,'Description','ykddot dynamics');
nlp.addPdeConstraint(Lpf, Kpf/Bpf*(geom.Lpf0 - Lpf) - F/Bpf, t, 'Method', IntegrationMethod, 'Description', 'Lpfdot dynamics');
nlp.addPdeConstraint(th, thdot, t, 'Method', IntegrationMethod,'Description','thdot dynamics');
nlp.addPdeConstraint(thdot, (F*geom.Lma*sin(phi) + m*g*geom.Ls*sin(gamma) - ...
    geom.Lf*cos(delth+geom.psi)*GRFy + geom.Lf*sin(delth+geom.psi)*GRFx)/I, t, ...
    'Method', IntegrationMethod,'Description','alpha dynamics');

%% Implement Constraints

% Trig constraints
nlp.addConstraint(0, phi, pi,'Description','Constrain Phi');
nlp.addConstraint(0, beta, pi,'Description','Constrain Beta');
nlp.addConstraint(0, gamma, pi,'Description','Constrain Gamma');
nlp.addConstraint(0, delth, pi, 'Description','Constrain delth');

% Unknown Parameter Constraints
nlp.addConstraint(0, cos(phi) - (Lpf^2 + geom.Lma^2 - geom.Ls^2)/(2*Lpf*geom.Lma), 0, 'Description','Define Phi');       % Define Phi
nlp.addConstraint(-.001, cos(beta) - (geom.Ls^2 + Lpf^2 - geom.Lma^2)/(2*geom.Ls*Lpf), 0.001, 'Description','Define Beta');      % Define Beta
nlp.addConstraint(0, sin(gamma) - (xk-xa)/geom.Ls, 0, 'Description','Define Gamma');                                                 % Define Gamma
nlp.addConstraint(0, delth - th+geom.th0, 0, 'Description','Define Del Theta');                                                             % Define deltaTheta
nlp.addConstraint(0, GRFy - m*ddyk - m*g + F*cos(beta+gamma), 0, 'Description','Define GRFy');                    % Define GRFy
nlp.addConstraint(0, GRFx + m*ddxk - F*sin(beta+gamma), 0, 'Description','Define GRFx');                    % Define GRFy

% Actuator Limits
nlp.addConstraint(0, F, Flim, 'Description','Force Limit');            % Force Limit
%nlp.addConstraint(-Vlim, dLpf, Vlim);   % Not sure how to add this. Creating a variable leaves no handle, yet creating a state requires omission of Ldot diff eq
%nlp.addConstraint(0, Lpf, Inf, 'Description','Length Constraint');

% Initial Conditions
%nlp.addConstraint(0, delth, 0, 'Description','Initial Condition: Delta Theta');
nlp.addConstraint(0, F.initial, 0, 'Description','Initial Condition: Force');                 % Initial Force
nlp.addConstraint(geom.th0, th.initial, geom.th0, 'Description','Initial Condition: theta');          % Initial ankle angle
nlp.addConstraint(0, thdot.initial, 0, 'Description','Initial Condition: thetadot');           % Initial ankle angular velocity
nlp.addConstraint(0, xa.initial, 0, 'Description','Initial Condition: Ankle-X');                % Initial ankle x-position
%nlp.addConstraint(geom.ya0, ya.initial, geom.ya0, 'Description','Initial Condition: Ankle-Y');        % Initial ankle y-position
nlp.addConstraint(0, ya.initial, 0, 'Description','Initial Condition: Ankle-Y');        % Initial ankle y-position
nlp.addConstraint(0, dxa.initial, 0, 'Description','Initial Condition: Ankle-X Vel');              % Initial joint velocities
nlp.addConstraint(0, dxk.initial, 0, 'Description','Initial Condition: Knee-X Vel');              %
nlp.addConstraint(0, dya.initial, 0, 'Description','Initial Condition: Ankle-Y Vel');              %
nlp.addConstraint(0, dyk.initial, 0, 'Description','Initial Condition: Knee-Y Vel');              %
nlp.addConstraint(geom.Lpf0, Lpf.initial, geom.Lpf0, 'Description','Initial Condition: PF Length');   % Initial F+PF length
%nlp.addConstraint(0,delth,0,'Description','Initial Condition: delta theta');

% Termination Conditions
nlp.addConstraint(geom.th0+.5236, th.final, geom.th0+.5236, 'Description','Termination Condition: Theta'); % Final ankle angle
nlp.addConstraint(0, thdot, Inf, 'Description','Termination Condition: Thetdot > 0');
nlp.addConstraint(0, thdot.final, 0, 'Description','Termination Condition: thetadot');                % Final ankle Velocity
nlp.addConstraint(0, t, 1, 'Description','Termination Condition: Time');                               % Target finish time

% Geometry Constraints
% nlp.addConstraint(0, xa-geom.Lf*cos(delth+geom.psi),0,'Description','Ankle x geometry');
% nlp.addConstraint(0, ya-geom.Lf*sin(delth+geom.psi),0,'Description','Ankle y geometry');
% nlp.addConstraint(0, xk-xa-geom.Ls*cos(th-geom.psi),0,'Description','Knee x geometry');
% nlp.addConstraint(0, yk-ya-geom.Ls*sin(th-geom.psi),0,'Description','Knee y geometry');
%nlp.addConstraint(0, ya, Inf, 'Description','Constrain ya to be positive');
%nlp.addConstraint(0, yk, Inf, 'Description','Constrain yk to be positive');
%nlp.addConstraint(geom.Ls^2, (xk-xa)^2+(yk-ya)^2, geom.Ls^2);  % Constrain ankle to knee by Ls [ CONSTRAINT IS BAD ]  
% nlp.addConstraint(0, Lf*sin(delth+psi) - ya, 0);   % Constrain toe to ground for stance phase

%% Define Objective and Solve

nlp.addObjective(trapz(F^2, t), 'Description','Minimize Integral force^2');
%nlp.addObjective(ya, 'Description','Drop');

optim = Ipopt(nlp);
optim.options.ipopt.max_iter = 5000;
optim.export;
optim.solve;

xasol = squeeze(eval(xa));
dxasol = squeeze(eval(dxa));
yasol = squeeze(eval(ya));
dyasol = squeeze(eval(dya));
xksol = squeeze(eval(xk));
dxksol = squeeze(eval(dxk));
ddxksol = squeeze(eval(ddxk));
yksol = squeeze(eval(yk));
dyksol = squeeze(eval(dyk));
ddyksol = squeeze(eval(ddyk));
thsol = squeeze(eval(th));
thdotsol = squeeze(eval(thdot));
Fsol = squeeze(eval(F));
GRFysol = squeeze(eval(GRFy));
GRFxsol = squeeze(eval(GRFx));
Lsol = squeeze(eval(Lpf));
phi_sol = squeeze(eval(phi));
beta_sol = squeeze(eval(beta));
gamma_sol = squeeze(eval(gamma));
delthsol = squeeze(eval(delth));
tsol = linspace(0, squeeze(eval(t)), nNodes);

%% Post-Processing

%% Plotting and Animation

response.time = tsol';
response.maxiter = length(tsol);
response.ankle = [xasol, yasol];
response.knee = [xksol, yksol];
response.theta = thsol;
response.thetadot = thdotsol;
response.Lpf = Lsol;
response.phi = phi_sol;
response.gamma = gamma_sol;
response.delth = delthsol;
response.beta = beta_sol;

% PHI IS NOT SOLVING PROPERLY

figure(1)
subplot(2,2,1)
plot(xasol, yasol, 'b'); hold on;
plot(xksol, yksol, 'k')
xlabel('X [m]'); ylabel('Y [m]');
legend('Ankle','Knee');
grid on;

subplot(2,2,2);
plot(dxasol, dyasol,'b');hold on;
plot(dxksol, dyksol, 'k');
xlabel('V_x [^m/_s]'); ylabel('V_y [^m/_s]')
legend('Ankle','Knee');
grid on;

subplot(2,2,3);
plot(tsol, thsol, tsol, thdotsol);
xlabel('Time [s]'); ylabel('\Theta [r] || \Thetadot [r/s]');
legend('\theta','\thetadot');
grid on;

subplot(2,2,4);
plot(tsol,Fsol, 'k'); hold on;
plot(tsol, GRFxsol, 'r');
plot(tsol, GRFysol, 'b');
xlabel('Time [s]'); ylabel('Force [N]');
legend('PF Force','GRFx','GRFy');
grid on;

figure(2)
plot(tsol, Lsol)
xlabel('Time [s]'); ylabel('PF Length [m]');

scene = PF_Scene(geom, response);
Player(scene);

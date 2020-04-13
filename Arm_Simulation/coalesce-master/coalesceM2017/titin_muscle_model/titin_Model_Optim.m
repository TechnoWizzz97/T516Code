% Titin Model placed within a 2d plantarflexor simulation
% Sum of torques is about the toe
% I believe some of the angles are backwards in software because of the way
% I drew the model & transcribed values to script

clc;
clear all;
close all;

Fmax = 101.4;          % [N] max isometric force ( source: Kiisa program )
L0 = 0.018;               % [m] Resting lenght of xm ( source: Kiisa program )
vmax = 10*L0;    % [m/s] max contraction velocity ( source: Kiisa )
Rp = L0/2;                   % [m] Radius of pulley ( source: Kiisa program [not sure if right param]  )
mp = 0.008;                % [kg] mass of pully ( source: Kiisa program )
Kss = 2369.7;           % [N/m] series-spring stiffness ( source: Kiisa Program )
Kts = 3754.5;            % [N/m] resistive spring stiffness ( source: Kiisa program )
Ip = 0.5*mp*Rp^2;      % [Nm2] Inertia of pulley ( arbitrary )
Bts = 0.3145;

m_r = 5;                   % Resistance mass [kg]
g = 9.81;                  % Gravitational acceleration [m/s2]
Q = 20;                    % F-L Curvature tuner

% ------------ Skeletal System -------------
alpha = deg2rad(130.751);       % [r] Forefoot-MA Angle
beta = deg2rad(34.622);           % [r] MA-Footprint Angle
gamma = deg2rad(14.626);      % [r] Footprint-Forefoot Angle
M = 15;                                      % [kg] Mass of full body
I = 1;                                          % [kgm2] Inertia of (????)
Ls = 0.5;                                    % [m] Length of shank
Lma = 4*.0254;                         % [m] Length of MA
Lfp = 12*.0254;                         % [m] Length of footprint
Lf = 9*.0254;                             % [m] Length of Forefoot

% Muscle Geometry Constraints
m_lim = (Ls + Lma)/2;
xplb = -m_lim; xpub = m_lim;
xtslb = -m_lim; xtsub = m_lim;
xmlb = -m_lim; xmub = m_lim;
xcelb = -m_lim; xceub = m_lim;
thlb = -2*pi; thub = 2*pi;

%IntegrationMethod = 'Trapezoidal';
IntegrationMethod = 'Implicit Euler';
%IntegrationMethod = 'Explicit Euler';

%%

nNodes = 51;

nlp = DirectCollocation(nNodes);

% Add Time
t = nlp.addTime;

%% Add States - PF MTU

xce = nlp.addState(1,-Inf, Inf, 'Description','State: Xce','Length', nNodes);
dxce = nlp.addState(1,-Inf, Inf, 'Description','State: Vel Xce','Length', nNodes);
xts = nlp.addState(1,-Inf, Inf, 'Description','State: Xts','Length', nNodes);
dxts = nlp.addState(1,-Inf, Inf, 'Description','State: Vel Xts','Length', nNodes);
xp = nlp.addState(1,-Inf, Inf, 'Description','State: Xp','Length', nNodes);
dxp = nlp.addState(1,-Inf, Inf, 'Description','State: Vel Xp','Length', nNodes);
xm = nlp.addState(1,-Inf, Inf, 'Description','State: Xm','Length', nNodes);
dxm = nlp.addState(1,-Inf, Inf, 'Description','State: Vel Xm','Length', nNodes);
th = nlp.addState(1,-Inf, Inf, 'Description','State: Th','Length', nNodes);
dth = nlp.addState(1,-Inf, Inf, 'Description','State: Vel Th','Length', nNodes);

%% Add States - Skeletal System

kx = nlp.addState(1, -Inf, Inf, 'Description','State: Knee x','Length',nNodes);
dkx = nlp.addState(1, -Inf, Inf, 'Description','State: Vel Knee x','Length',nNodes);
ky = nlp.addState(1, -Inf, Inf, 'Description','State: Knee y','Length',nNodes);
dky = nlp.addState(1, -Inf, Inf, 'Description','State: Vel Knee y','Length',nNodes);
tx = nlp.addState(1, -Inf, Inf, 'Description','State: Toe x','Length',nNodes);
dtx = nlp.addState(1, -Inf, Inf, 'Description','State: Vel Toe x','Length',nNodes);
ty = nlp.addState(1, -Inf, Inf, 'Description','State: Toe y','Length',nNodes);
dty = nlp.addState(1, -Inf, Inf, 'Description','State: Vel Toe y','Length',nNodes);
sigma = nlp.addState(1,-Inf, Inf, 'Description','State: Foot Angle','Length',nNodes);
dsigma = nlp.addState(1, -Inf, Inf, 'Description','State: Vel Foot Angle','Length',nNodes);

%% Add Inputs
A = nlp.addInput(.5, 0, 1, 'Description', 'Input: Muscle Activation','Length', nNodes);

%% Add Variables - PF MTU
Bce = nlp.addVariable(1, 0, Inf, 'Description','Variable: CE Damping Component','Length',nNodes);
Fss = nlp.addVariable(1, 0, Inf, 'Description','Variable: Tendon Force','Length',nNodes);
C_met = nlp.addVariable(1, 0, Inf, 'Description','Variable: Metabolic Cost','Length',nNodes);
f_met1 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 1','Length',nNodes);
f_met2 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 2','Length',nNodes);

e_fit1 = 0.11; %0.15
off_fit1 = -0.08; %-0.05
e_fit2 = 0.2; % 0.01;

Lss = xm-xp;
Ldot = dxm-dxp;
Fss = Kss*Lss;   % Comment for nonlinear SEE stiffness

%% Add Variables - Skeletal System
phi = nlp.addVariable(1, 0, pi,'Description','PF-Ankle Angle', 'Length',nNodes);
Frx = nlp.addVariable(1, -Inf, Inf, 'Description','GRF - x', 'Length',nNodes);
Fry = nlp.addVariable(1, 0, Inf, 'Description','GRF - y', 'Length',nNodes);
theta = nlp.addVariable(1, 0, pi, 'Description','Shank Angle', 'Length',nNodes);
Lpf = nlp.addVariable(1, 0, Lma+Ls, 'Description','PF Length', 'Length',nNodes);
mu = 2*pi-alpha-theta;          % MA-Shank angle
hx = tx - Lfp*cos(sigma);       % heel x
hy = ty + Lfp*sin(sigma);      % heel y
sigmaddot = (M*g*(-Lf*cos(sigma+gamma) + Ls*cos(sigma+gamma+(pi-theta)) - Fss*sin(beta+phi)))/I;
kxddot = (Fss*cos(sigma+pi-phi-beta)-Frx)/M;
kyddot = (Fss*sin(sigma+pi-phi-beta)+Fry-M*g)/M;

%% Add Dynamics - PF MTU
nlp.addPdeConstraint(xce, dxce, t,'Method', IntegrationMethod, 'Description', 'xce dynamics');
nlp.addPdeConstraint(xts, dxts, t, 'Method', IntegrationMethod, 'Description', 'xts dynamics');
nlp.addPdeConstraint(xm, dxm, t, 'Method', IntegrationMethod, 'Description', 'xm dynamics');

nlp.addPdeConstraint(dxm, -Lfp*sigmaddot, t, 'Method', IntegrationMethod, 'Description', 'dxm dynamics');

Fce = Fmax*A;

nlp.addPdeConstraint(xp, dxp, t, 'Method', IntegrationMethod,'Description','xp dot dynamics');
nlp.addPdeConstraint(dxp, (Fss - (Fce + Bce*dxce + Kts*xts + Bts*dxts))/mp, t, ...
    'Method', IntegrationMethod,'Description','xp ddot dynamics');

nlp.addPdeConstraint(th, dth, t, 'Method', IntegrationMethod,'Description','theta dot dynamics');
nlp.addPdeConstraint(dth, ((Rp*Fce + Rp*Bce*dxce) - (Rp*Kts*xts + Rp*Bts*dxts))/Ip, t, ...
    'Method', IntegrationMethod,'Description','theta ddot dynamics');

%% Add Dynamics - Skeletal System
nlp.addPdeConstraint(kx, dkx, t, 'Method',IntegrationMethod,'Description','kx dynamics');
nlp.addPdeConstraint(dkx, kxddot, t,  'Method', IntegrationMethod,'Description','dkx dynamics');

nlp.addPdeConstraint(ky, dky, t, 'Method', IntegrationMethod,'Description','ky dynamics');
nlp.addPdeConstraint(dky, kyddot, t,  'Method', IntegrationMethod,'Description','dky dynamics');

nlp.addPdeConstraint(tx, dtx, t, 'Method', IntegrationMethod,'Description','tx dynamics');
%nlp.addPdeConstraint(dtx, 0, t, 'Method', IntegrationMethod,'Description','dtx dynamics');

nlp.addPdeConstraint(ty, dty, t, 'Method', IntegrationMethod,'Description','ty dynamics');
%nlp.addPdeConstraint(dty, 0, t, 'Method', IntegrationMethod,'Description','dty dynamics');

nlp.addPdeConstraint(sigma, dsigma, t, 'Method',IntegrationMethod,'Description','sigma dynamics');
nlp.addPdeConstraint(dsigma, sigmaddot, t, 'Method', IntegrationMethod,'Description','dsigma dynamics');


%% Implement Constraints
eps = 1e-5;
eps2 = 1e-3;

% Constraint Knee to Toe
nlp.addConstraint(0, kx - tx + Lf*cos(sigma+gamma) - Ls*cos(sigma+gamma+(pi-theta)), 0,  'Description','Constrain knee to toe - X');
nlp.addConstraint(0, ky- ty -Lf*sin(sigma+gamma) - Ls*sin(sigma+gamma+(pi-theta)), 0, 'Description','Constraint knee to toe - Y');

% Constrain phi trigonometrically
nlp.addConstraint(0, sin(phi)-Ls/Lpf*sin(mu), 0, 'Description','Constrain Phi');

% Constrain Lpf geometrically
nlp.addConstraint(0, Lpf^2 - (kx-hx)^2 - (ky-hy)^2, 0, 'Description','Constrain Lpf');

% Enforce Reaction Forces
nlp.addConstraint(-eps, Fry - M*g + Fss*sin(sigma+pi-phi-beta)-M*kyddot, eps, 'Description','Enforce Normal Force');
nlp.addConstraint(-eps, Frx + M*kxddot - Fss*cos(sigma+pi-phi-beta), eps, 'Description','Enforce GRF - x');

% Binary Contractile element damping
nlp.addConstraint(-eps, Bce - ( (36.143-5.4706)*1/(1+exp(100*dxm)) + 5.4706), eps, 'Description','Create "Binary" CE damping');

% Nonlinear SEE stiffness
%nlp.addConstraint(0, -Lss + 1/Kss*(Fss + Fmax/Q*log(1-0.9*exp(-Q/Fmax*Fss)) - Fmax/Q*log(0.1)), 0, 'Description','Establish Tendon F-L Relationship');

% Metabolic Cost
nlp.addConstraint(-eps, f_met1 - (0.22*0.5.*(-(dxce/vmax-off_fit1)+((dxce/vmax-off_fit1).^2+e_fit1^2).^(1/2))./((dxce/vmax-off_fit1).^2+e_fit1^2).^(1/2)), eps, 'Description','Met Cost Cmp 1');
nlp.addConstraint(-eps, f_met2 - (0.01+(0.11*(dxce/vmax))*0.5.*((dxce/vmax)+((dxce/vmax).^2+e_fit2^2).^(1/2))./((dxce/vmax).^2+e_fit2^2).^(1/2)), eps, 'Description','Met Cost cmp 2');
nlp.addConstraint(-eps, C_met - (f_met1 + f_met2), eps, 'Description','Relate Metabolic Cost to dxce');

% Geometry Constraints
nlp.addConstraint(xplb, xp, xpub,'Description','Bound Xp');
nlp.addConstraint(xtslb, xts, xtsub, 'Description','Bound Xts');
nlp.addConstraint(xmlb, xm, xmub, 'Description','Bound Xm');
nlp.addConstraint(xcelb, xce, xceub, 'Description','Bound Xce');
nlp.addConstraint(thlb, th, thub, 'Description','Bound Theta');

% Pulley Constraints
nlp.addConstraint(0, xts-xp-Rp*th, 0,'Description','Relate xt to xp to th');
nlp.addConstraint(0, xce-xp+Rp*th, 0, 'Description','Relate xce to xp to th');

% Actuator Limits
nlp.addConstraint(0, A, 1,'Description','Activation Limit');

nlp.addConstraint(-vmax, dxce, vmax, 'Description','Velocity Constraint');

% Anchor Toe
nlp.addConstraint(0, tx, 0, 'Description','Anchor toe - x');
nlp.addConstraint(0, ty, 0, 'Description','Anchor toe - y');

% Initial Conditions - PF MTU
nlp.addConstraint(0, xce.initial, 0,'Description','Initial Condition: Xce');
nlp.addConstraint(0, dxce.initial, 0, 'Description','Initial Condition: dXce');
nlp.addConstraint(0, xts.initial, 0, 'Description','Initial Condition: Xts');
nlp.addConstraint(0, dxts.initial, 0, 'Description','Initial Condition: dXts');
nlp.addConstraint(0, xp.initial, 0, 'Description','Initial Condition: Xp');
nlp.addConstraint(0, dxp.initial, 0, 'Description','Initial Condition: dXp');
nlp.addConstraint(0, xm.initial, 0, 'Description','Initial Condition: Xm');
nlp.addConstraint(0, dxm.initial, 0, 'Description','Initial Condition: Xm');
nlp.addConstraint(0, th.initial, 0,'Description','Initial Condition: Theta');
nlp.addConstraint(0, dth.initial, 0,'Description','Initial Conditions: Theta Dot');

% Initial Conditions - Skeletal System
nlp.addConstraint(0, sigma.initial, 0, 'Description','Initial Conditions: Sigma');

% Termination Conditions
nlp.addConstraint(pi/4, sigma.final, pi/4, 'Description','Termination Condition: sigma');
%nlp.addConstraint(0, Fry.final, 0);
nlp.addConstraint(0, t.final, 1, 'Description','Termination Condition: Time');

%% Define Objective and Solve

%nlp.addObjective(trapz((A*Fmax)^2, t), 'Description','Minimize CE force^2');
nlp.addObjective(trapz(C_met^2, t), 'Description','Minimize Metabolic Cost');
%nlp.addObjective(t, 'Description','Minimize Time');

optim = Ipopt(nlp);
%optim.options.ipopt.max_iter = 5000;
optim.export;
optim.solve;

%%

tsol = linspace(0, squeeze(eval(t)), nNodes);
xcesol = squeeze(eval(xce));
dxcesol = squeeze(eval(dxce));
xtssol = squeeze(eval(xts));
dxtssol = squeeze(eval(dxts));
xpsol = squeeze(eval(xp));
dxpsol = squeeze(eval(dxp));
thsol = squeeze(eval(th));
dthsol = squeeze(eval(dth));
xmsol = squeeze(eval(xm));
dxmsol = squeeze(eval(dxm));
Asol = squeeze(eval(A));
Bcesol = squeeze(eval(Bce));
Tsol = squeeze(eval(Fss));
Fcesol = squeeze(eval(Fce));
Lsssol = squeeze(eval(Lss));
Ldotsol = squeeze(eval(Ldot));
Fsssol = squeeze(eval(Fss));
C_metsol = squeeze(eval(C_met));

sigmasol = squeeze(eval(sigma));
dsigmasol = squeeze(eval(dsigma));
thetasol = squeeze(eval(theta));
phisol = squeeze(eval(phi));
Lpfsol = squeeze(eval(Lpf));
kxsol = squeeze(eval(kx));
kysol = squeeze(eval(ky));
dkxsol = squeeze(eval(dkx));
dkysol = squeeze(eval(dky));
txsol = squeeze(eval(tx));
tysol = squeeze(eval(ty));
hxsol = squeeze(eval(hx));
hysol = squeeze(eval(hy));
Frxsol = squeeze(eval(Frx));
Frysol = squeeze(eval(Fry));


Fout = Kss*(xmsol-xpsol);
stress = Fout / pi*Rp^2;
strain = xmsol/(2*xmub);

response.time = tsol';
response.maxiter = length(tsol);
response.xm = xmsol;
response.xce = xcesol;
response.xp = xpsol;
response.xts = xtssol;
response.theta = thsol;

L0 = 2*xmub;
geom.L = L0+.018;
geom.R = L0/2;

%% Plotting and Animation

figure(1)
suptitle('Titin Model Minimum Energy Behavior');
subplot(2,2,1)
plot(tsol, xcesol, 'b'); hold on;
plot(tsol, xpsol,'m');
plot(tsol, xtssol,'r')
plot(tsol, xmsol, 'k')
xlabel('Time [s]'); ylabel('X [m]');
legend('CE','P','TS','M','Location','southwest');
grid on;

subplot(2,2,2);
plot(tsol, dxcesol,'b');hold on;
plot(tsol, dxpsol,'m');
plot(tsol, dxtssol,'r');
plot(tsol, dxmsol, 'k');
xlabel('Time'); ylabel('V [^m/_s]')
legend('CE','P','TS','M','Location','southwest');
grid on;

subplot(2,2,3);
plot(tsol, thsol, tsol, dthsol);
xlabel('Time [s]'); ylabel('\Theta [r] || \Thetadot [r/s]');
legend('\theta','\thetadot','location','northwest');
grid on;

subplot(2,2,4);
plot(tsol, Asol, 'r--'); hold on;
plot(tsol, Tsol/Fmax, 'b');
xlabel('Time [s]'); ylabel('Force [N]');
legend('Activation','Output Force (normalized)','Location','southeast');
grid on;

% -----------------------------------------

Wt = zeros(length(tsol), 1);
for i = 2:length(tsol)
    Wt(i) = sum(Asol(1:i)*Fmax.*-dxcesol(1:i)*(tsol(i)-tsol(i-1)));
end % Total Work

l = 1; s = 1;
for i = 1:length(xmsol)
    if dxmsol(i) >= 0
        xlen(l) = xmsol(i);
        Wtlen(l) = Wt(i);
        Llen(l) = Lsssol(i);
        Fsslen(l) = Fsssol(i);
        l=l+1;
    elseif dxmsol(i) < 0
        xsho(s) = xmsol(i);
        Wtsho(s) = Wt(i);
        Lsho(s) = Lsssol(s);
        Fsssho(s) = Fsssol(s);
        s=s+1;
    end
end

figure(2)

subplot(3,1,1);
plot(tsol, xmsol);
xlabel('Time [s]'); ylabel('Xm [m]');
grid on;

subplot(3,1,2);
plot(tsol, Fout);
xlabel('Time [s]'); ylabel('F_o_u_t');
grid on;

subplot(3,1,3);
plot(tsol, Wt); hold on;
xlabel('Time [s]'); ylabel('Work [J]');
grid on;

% ---------------------------------------
%%
figure(3)

% F-L Relationship
%{
F_vec = linspace(0,Fmax,1e3);
L_vec = 1/Kss*(F_vec + Fmax/Q*log(1-0.9*exp(-Q/Fmax*F_vec)) - Fmax/Q*log(0.1));

for i = 1:length(tsol)
    clf;
    plot(L_vec, F_vec, 'k'); hold on;
    if Ldotsol(i) >= 0
        scatter(Lsssol(i), Fsssol(i), 'ro', 'LineWidth', 5);
    elseif Ldotsol(i) < 0
        scatter(Lsssol(i), Fsssol(i), 'bo','LineWidth', 5);
    end
    title('F-L Relationship');
    xlabel('\DeltaLength [m]'); ylabel('Force [N]');
    grid on
    
    pause(0.01);
end
%}

%Metabolic Cost C-V
v = linspace(-vmax, vmax, 1000);
e_fit1 = 0.11; %0.15
off_fit1 = -0.08; %-0.05
f_met_n_fit1 = 0.22*0.5.*(-(v/vmax-off_fit1)+((v/vmax-off_fit1).^2+e_fit1^2).^(1/2))./((v/vmax-off_fit1).^2+e_fit1^2).^(1/2);
e_fit2 = 0.2; % 0.01;
f_met_n_fit2 = 0.01+(0.11*(v/vmax))*0.5.*((v/vmax)+((v/vmax).^2+e_fit2^2).^(1/2))./((v/vmax).^2+e_fit2^2).^(1/2);

for i = 1:length(tsol)
    clf;
    p3 = plot(v/vmax, f_met_n_fit1+f_met_n_fit2, 'k'); hold on;
    gcf = p3;
    if Ldotsol(i) >= 0
        scatter(dxcesol(i)/vmax, C_metsol(i), 'ro', 'LineWidth', 5);
    elseif Ldotsol(i) < 0
        scatter(dxcesol(i)/vmax, C_metsol(i), 'bo','LineWidth', 5);
    end
    title('Metabolic Cost');
    xlabel('CE Velocity'); ylabel('Metabolic Cost');
    grid on
    
    pause(0.01);
end

%%

figure(4)
for i = 1:length(tsol)
    clf;
    plot(txsol(i), tysol(i),'ok'); hold on;
    plot(hxsol(i), hysol(i),'ro');
    plot(kxsol(i), kysol(i), 'bo');
    plot([txsol(i) txsol(i)-Lf*cos(sigmasol(i)+gamma)], [tysol(i) tysol(i)+Lf*sin(sigmasol(i)+gamma)],'k');
    plot([txsol(i)-Lf*cos(sigmasol(i)+gamma), txsol(i)-Lf*cos(sigmasol(i)+gamma)+Ls*cos(sigmasol(i)+gamma+pi-thetasol(i))], ...
        [tysol(i)+Lf*sin(sigmasol(i)+gamma), tysol(i)+Lf*sin(sigmasol(i)+gamma)+Ls*sin(sigmasol(i)+gamma+pi-thetasol(i))],'b');
    axis([-0.35, 0.35, 0, 0.7]);
    grid on;
    pause(0.01);
end
%%

% if optim.info.status == 0 || optim.info.status == 1
%     scene = Titin_Scene(geom, response);
%     Player(scene);
% end

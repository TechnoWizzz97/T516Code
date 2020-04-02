clc;
clear all;
close all;

Fmax = 101.4;          % [N] max isometric force ( source: Kiisa program )
L0 = 0.018;               % [m] Resting lenght of xm ( source: Kiisa program )
vmax = 10*L0;          % [m/s] max contraction velocity ( source: Kiisa )
R = L0/2;                   % [m] Radius of pulley ( source: Kiisa program [not sure if right param]  )
m = 5*0.008;                % [kg] mass of pully ( source: Kiisa program )
Kss = 2369.7;            % [N/m] series-spring stiffness ( source: Kiisa Program )
Kts = 3754.5;        
Ip = 0.5*m*R^2;      % [Nm2] Inertia of pulley ( arbitrary )
Bts = 0.3145;

m_r = 5;                   % Resistance mass [kg]
g = 9.81;                  % Gravitational acceleration [m/s2]
Q = 20;                    % F-L Curvature tuner

% Geometry Constraints    % [N/m] resistive spring stiffness ( source: Kiisa program )
xplb = -0.05; xpub = 0.05;
xtslb = -0.05; xtsub = 0.05;
xmlb = -0.05; xmub = 0.05;
xcelb = -0.05; xceub = 0.05;
thlb = -2*pi; thub = 2*pi;

xmfin = -0.02;

%IntegrationMethod = 'Trapezoidal';
IntegrationMethod = 'Implicit Euler';
%IntegrationMethod = 'Explicit Euler';

%%

nNodes = 1001;

nlp = DirectCollocation(nNodes);

% Add Time
t = nlp.addTime;

%% Add States [ xce, dxce, xts, dxts, xp, dxp, th, dth, xm, dxm]
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

%% Add Inputs
A = nlp.addInput(.5, 0, 1, 'Description', 'Input: Muscle Activation','Length', nNodes);

%% Add Variables
Bce = nlp.addVariable(1, 0, Inf, 'Description','Variable: CE Damping Component','Length',nNodes);
%Fss = nlp.addVariable(1, 0, Inf, 'Description','Variable: Tendon Force','Length',nNodes);
C_met = nlp.addVariable(1, 0, Inf, 'Description','Variable: Metabolic Cost','Length',nNodes);
f_met1 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 1','Length',nNodes);
f_met2 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 2','Length',nNodes);

e_fit1 = 0.11; %0.15
off_fit1 = -0.08; %-0.05
e_fit2 = 0.2; % 0.01;

Lss = xm-xp;
Ldot = dxm-dxp;
Fss = Kss*Lss;

%% Add dynamics
nlp.addPdeConstraint(xce, dxce, t,'Method', IntegrationMethod, 'Description', 'xce dynamics');
nlp.addPdeConstraint(xts, dxts, t, 'Method', IntegrationMethod, 'Description', 'xts dynamics');
nlp.addPdeConstraint(xm, dxm, t, 'Method', IntegrationMethod, 'Description', 'xm dynamics');

nlp.addPdeConstraint(dxm, g-Fss/m_r, t, 'Method',IntegrationMethod, 'Description','Dxm dynamics');

Fce = Fmax*A;

nlp.addPdeConstraint(xp, dxp, t, 'Method', IntegrationMethod,'Description','xp dot dynamics');
nlp.addPdeConstraint(dxp, (Fss - (Fce + Bce*dxce + Kts*xts + Bts*dxts))/m, t, ...
    'Method', IntegrationMethod,'Description','xp ddot dynamics');

nlp.addPdeConstraint(th, dth, t, 'Method', IntegrationMethod,'Description','theta dot dynamics');
nlp.addPdeConstraint(dth, ((R*Fce + R*Bce*dxce) - (R*Kts*xts + R*Bts*dxts))/Ip, t, ...
    'Method', IntegrationMethod,'Description','theta ddot dynamics');

%% Implement Constraints

eps = 1e-5;
eps2 = 1e-3;

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
nlp.addConstraint(0, xts-xp-R*th, 0,'Description','Relate xt to xp to th');
nlp.addConstraint(0, xce-xp+R*th, 0, 'Description','Relate xce to xp to th');

% Actuator Limits
nlp.addConstraint(0, A, 1,'Description','Activation Limit');
nlp.addConstraint(-vmax, dxce, vmax, 'Description','Velocity Constraint');

% Initial Conditions
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

% Termination Conditions
nlp.addConstraint(0, dxm.final, 0, 'Description','Termination Condition: dxm');
nlp.addConstraint(0, dxce.final, 0,'Description','Termination Condition: dxce');
nlp.addConstraint(0, dxts.final, 0,'Description','Termination Condition: dxts');
nlp.addConstraint(0, dxp.final, 0, 'Description','Termination Condition: dxp');
nlp.addConstraint(0, dth.final, 0, 'Description','Termination Condition: Theta Dot');
nlp.addConstraint(xmfin, xm.final, xmfin, 'Description','Termination Condition: xm');
nlp.addConstraint(0, t.final, 1, 'Description','Termination Condition: Time');

%% Define Objective and Solve

%nlp.addObjective(trapz((A*Fmax)^2, t), 'Description','Minimize CE force^2');
nlp.addObjective(trapz(C_met^2, t), 'Description', 'Minimize Metabolic Cost');

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
Fcesol = squeeze(eval(Fce));
Lsssol = squeeze(eval(Lss));
Ldotsol = squeeze(eval(Ldot));
Fsssol = squeeze(eval(Fss));
C_metsol = squeeze(eval(C_met));

Fout = Kss*(xmsol-xpsol);
stress = Fout / pi*R^2;
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
plot(tsol, Fsssol/Fmax, 'b');
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
    plot(v/vmax, f_met_n_fit1+f_met_n_fit2, 'k'); hold on;
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

if optim.info.status == 0 || optim.info.status == 1
    scene = Titin_Scene(geom, response);
    Player(scene);
end

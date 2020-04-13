clc;
clear all;
close all;

Fmax = 101.4;          % [N] max isometric force ( source: Kiisa program )
L0 = 0.018;               % [m] Resting lenght of xm ( source: Kiisa program )
vmax = 500*L0;          % [m/s] max contraction velocity ( source: Kiisa )
R = L0/2;                   % [m] Radius of pulley ( source: Kiisa program [not sure if right param]  )
m = 0.008;                % [kg] mass of pully ( source: Kiisa program )
Kss = 2369.7;            % [N/m] series-spring stiffness ( source: Kiisa Program )
Kts = 3754.5;             % [N/m] resistive spring stiffness ( source: Kiisa program )
%Kten = 50.580*1000; % [N/m] Tendon Stiffness ( source: Buchanan Marsh 2001 || control stiffness ) 
Kten = 77.808*1000; % [N/m] Tendon Stiffness ( source: Buchanan Marsh 2001 || Level-Running stiffness ) 
%Kten = 88.054*1000; % [N/m] Tendon Stiffness ( source: Buchanan Marsh 2001 || Downhill-Running stiffness ) 
Ip = 0.5*m*R^2;       % [Nm2] Inertia of pulley ( arbitrary )
Bts = 0.3145;             % [Ns/m] Damping of titin-spring
apen = deg2rad(20); % [r] Pennation Angle ( source: Daley Biewiener 2003 )
Keff = 1/(1/Kss + 1/Kten);

g = 9.81;                  % Gravitational acceleration [m/s2]
Q = 20;                    % F-L Curvature tuner for tendon stiffness

M = 1.25;                  % [kg] Mass of FrankenFowl
rh = 0.005;               % [m] Lever Arm of heel
rf = rh;                 % [m] Lever arm of forefoot

mlim = 0.05;
% Geometry Constraints    
xplb = -mlim; xpub = mlim;
xtslb = -mlim; xtsub = mlim;
xmlb = -mlim; xmub = mlim;
xcelb = -mlim; xceub = mlim;
thlb = -2*pi; thub = 2*pi;

IntegrationMethod = 'Trapezoidal';
%IntegrationMethod = 'Implicit Euler';
%IntegrationMethod = 'Explicit Euler';

%%

nNodes = 91;

nlp = DirectCollocation(nNodes);

% Add Time
t = nlp.addTime;

%% Add States - Muscle Unit
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

%% Add States - FrankenFowl
yf = nlp.addState(1, -Inf, Inf, 'Description','State: FF - Y','Length',nNodes);
dyf = nlp.addState(0, -Inf, Inf, 'Description','State: FF - Vel Y','Length',nNodes);

%% Add Inputs
A = nlp.addInput(0.5, 0, 1, 'Description', 'Input: Muscle Activation','Length', nNodes);

%% Add Variables
Bce = nlp.addVariable(1, 0, Inf, 'Description','Variable: CE Damping Component','Length',nNodes);

% ---- ------ ----- ----- ----- ----- ------ ------ ------ Metabolic Cost ----- ------ ------ ------ ----- ------ ------ ------ ----- -----
C_met = nlp.addVariable(1, 0, Inf, 'Description','Variable: Metabolic Cost','Length',nNodes);
f_met1 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 1','Length',nNodes);
f_met2 = nlp.addVariable(1, 0, Inf, 'Description','Variable: Met Cost cmp 2','Length',nNodes);
e_fit1 = 0.11; %0.15
off_fit1 = -0.08; %-0.05
e_fit2 = 0.2; % 0.01;
% ----- ----- ----- ----- ----- ----- ------ ------ ------ ----- ------ ------ ------ ----- ------ ------ ------ ----- ------ ------ ------ ----- -----

Lss = xm-xp;
Ldot = dxm-dxp;
Fm = Kss*Lss;
Fg = nlp.addVariable(1, 0, Inf, 'Description','Variable: GRF','Length',nNodes);
sphi = nlp.addVariable(1, 0, 1, 'Description','Variable: Ankle Angle - sin','Length',nNodes);
cphi = nlp.addVariable(1, 0, 1, 'Description','Variable: Ankle Angle - cos','Length',nNodes);

%% Add Dynamics - Muscle Unit
%yfddot = (Fg+Fm-M*g)/M;
yfddot = (Fg-Fm-M*g)/M;

nlp.addPdeConstraint(xce, dxce, t,'Method', IntegrationMethod, 'Description', 'xce dynamics');
nlp.addPdeConstraint(xts, dxts, t, 'Method', IntegrationMethod, 'Description', 'xts dynamics');

nlp.addPdeConstraint(xm, dxm, t, 'Method', IntegrationMethod, 'Description', 'xm dynamics');
%nlp.addPdeConstraint(dxm, xmddot, t, 'Method',IntegrationMethod, 'Description','dxm dynamics');

Fce = Fmax*A;

nlp.addPdeConstraint(xp, dxp, t, 'Method', IntegrationMethod,'Description','xp dot dynamics');
nlp.addPdeConstraint(dxp, (Fm - (Fce + Bce*dxce + Kts*xts + Bts*dxts))/m, t, ...
    'Method', IntegrationMethod,'Description','xp ddot dynamics');

nlp.addPdeConstraint(th, dth, t, 'Method', IntegrationMethod,'Description','theta dot dynamics');
nlp.addPdeConstraint(dth, ((R*Fce + R*Bce*dxce) - (R*Kts*xts + R*Bts*dxts))/Ip, t, ...
    'Method', IntegrationMethod,'Description','theta ddot dynamics');

%% Add Dynamics - FrankenFowl
nlp.addPdeConstraint(yf, dyf, t, 'Method', IntegrationMethod, 'Description','yf Dynamics');
nlp.addPdeConstraint(dyf, yfddot, t, 'Method', IntegrationMethod, 'Description','dyf Dynamics');

%% Implement Constraints

eps = 1e-5;
eps2 = 1e-4;
eps3 = 1e-3;
eps4 = 1e-2;

% ----- ----- ----- ----- ----- ----- ------ ------ MTU Constraints ------ ------ ----- ------ ------ ------ ----- ------ ------ ------ ----- -----
% Binary Contractile element damping
nlp.addConstraint(-eps2, Bce - ((36.143-5.4706)*1/(1+exp(100*dxce)) + 5.4706), eps2, 'Description','Create "Binary" CE damping');

% Nonlinear tendon stiffness
%nlp.addConstraint(0, -xm + 1/Kss*(Fm + Fmax/Q*log(1-0.9*exp(-Q/Fmax*Fm)) - Fmax/Q*log(0.1)), 0, 'Description','Establish Tendon F-L Relationship');

% Metabolic Cost
nlp.addConstraint(0, f_met1 - (0.22*0.5.*(-(dxce/vmax-off_fit1)+((dxce/vmax-off_fit1).^2+e_fit1^2).^(1/2))./((dxce/vmax-off_fit1).^2+e_fit1^2).^(1/2)), 0, 'Description','Met Cost Cmp 1');
nlp.addConstraint(0, f_met2 - (0.01+(0.11*(dxce/vmax))*0.5.*((dxce/vmax)+((dxce/vmax).^2+e_fit2^2).^(1/2))./((dxce/vmax).^2+e_fit2^2).^(1/2)), 0, 'Description','Met Cost cmp 2');
nlp.addConstraint(0, C_met - (f_met1 + f_met2), 0, 'Description','Relate Metabolic Cost to dxce');

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
nlp.addConstraint(-vmax, dxts, vmax, 'Description','Velocity Constraint');

% ----- ----- ----- ----- ----- ----- ------ ----- ------ Franken-Fowl ------ ----- ------ ------ ------ ----- ------ ------ ------ ----- ----- -----
% Geometrically constrain yf
nlp.addConstraint(0, yf - rf*sphi, 0, 'Description','Constrain yf geometrically');
nlp.addConstraint(0, rf^2 - ((rf*cphi)^2 + yf^2), 0, 'Description','Define cos phi');

% Constrain Fg
%nlp.addConstraint(0, Fg*(rf/2)*cphi + M*g*(rf/2)*cphi - Fm*(rh+rf/2)*cphi, 0, 'Description','Define GRF - Moment abt forefoot midpoint');  
nlp.addConstraint(0, Fg*(rf/2) + M*g*(rf/2) - Fm*(rh+rf/2), 0, 'Description','Define GRF - Moment abt forefoot midpoint');  

% Constrain Force Production
nlp.addConstraint(0, Fg, Inf, 'Description', 'Unilateral Contact Force');
nlp.addConstraint(0, Fm, Inf, 'Description', 'No ss compression');

% Constrain y to be > 0
nlp.addConstraint(0, yf, Inf, 'Description','Keep above ground');

% Initial Conditions - Muscle Unit at rest
% nlp.addConstraint(0, xce.initial, 0,'Description','Initial Condition: Xce');
% nlp.addConstraint(0, dxce.initial, 0, 'Description','Initial Condition: dXce');
% nlp.addConstraint(0, xts.initial, 0, 'Description','Initial Condition: Xts');
% nlp.addConstraint(0, dxts.initial, 0, 'Description','Initial Condition: dXts');
% nlp.addConstraint(0, xp.initial, 0, 'Description','Initial Condition: Xp');
% nlp.addConstraint(0, dxp.initial, 0, 'Description','Initial Condition: dXp');
% nlp.addConstraint(0, xm.initial, 0, 'Description','Initial Condition: Xm');
% nlp.addConstraint(0, dxm.initial, 0, 'Description','Initial Condition: Xm');
% nlp.addConstraint(0, th.initial, 0,'Description','Initial Condition: Theta');
% nlp.addConstraint(0, dth.initial, 0,'Description','Initial Conditions: Theta Dot');

% Initial Contidion - FrankenFowl
%nlp.addConstraint(0, yf.initial, rf/8, 'Description','Initial Condition: yf');
nlp.addConstraint(0, dyf.initial, 0, 'Description','Initial Condition: dyf');
%nlp.addConstraint(0, Fg.initial - M*g, 0, 'Description','Initial Condition: GRF and Acceleration');
% nlp.addConstraint(0, dyf.initial, Inf, 'Description','Initial Condition: Enforce Vertical Motion');
% nlp.addConstraint(0, dyf.initial, 0, 'Description','Initial Conditions: dyf');
% nlp.addConstraint(0, A.initial, 0, 'Description','Initial Conditions: A');
% nlp.addConstraint(0, Ften.initial, 0, 'Description','Initial Conditions: Ften');
nlp.addConstraint(M*g, Fg.initial, M*g, 'Description','Initial Conditions: Fg');

 h = 0.025;
 tapex = dyf.final/g;
 
% Termination Conditions
%nlp.addConstraint(rf/4, yf.final, Inf, 'Description', 'Termination Condition: y-pos');
%nlp.addConstraint(eps2, yf.final-yf.initial, eps2, 'Description', 'Termination Condition: y-pos');
%nlp.addConstraint(0, dyf, Inf, 'Description','Termination Condition: Takeoff - Acceleration');
nlp.addConstraint(0, dyf.final, Inf, 'Description','Termination Condition: Takeoff - Velocity');
nlp.addConstraint(0, 2*g*h - dyf.final^2, 0, 'Description','Termination Condition: Takeoff - Projectile Motion');
nlp.addConstraint(0, Fg.final, 0, 'Description','Termination Condition: Takeoff - Force');
nlp.addConstraint(eps, t.final, 1, 'Description','Termination Condition: Time');

%% Define Objective and Solve
%nlp.addObjective(trapz((A*Fmax)^2, t), 'Description','Minimize CE force^2');
%nlp.addObjective(trapz(A^2, t), 'Description','Minimize CE force^2');
nlp.addObjective(trapz(C_met, t), 'Description', 'Minimize Metabolic Cost');
%nlp.addObjective(trapz(Fg, t), 'Description', 'Minimize GRF');
%nlp.addObjective(trapz(yfddot, t), 'Description', 'Minimize Acceleration');
%nlp.addObjective(0, 'Description', 'No objective. Solving dynamics');
%nlp.addObjective(t,'Description','Minimize Time');

optim = Ipopt(nlp);
optim.options.ipopt.max_iter = 5000;
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
Fmsol = squeeze(eval(Fm));
%Ftensol = squeeze(eval(Ften));
C_metsol = squeeze(eval(C_met));
yddotsol = squeeze(eval(yfddot));
yfsol = squeeze(eval(yf));
dyfsol = squeeze(eval(dyf));
sphisol = squeeze(eval(sphi));
cphisol = squeeze(eval(cphi));
%dphisol = squeeze(eval(dphi));
Fgsol = squeeze(eval(Fg));
%xmddotsol = squeeze(eval(xmddot));
apex_t = squeeze(eval(tapex));

musc_response.time = tsol';
musc_response.maxiter = length(tsol);
musc_response.xm = xmsol;
musc_response.xce = xcesol;
musc_response.xp = xpsol;
musc_response.xts = xtssol;
musc_response.theta = thsol;

L0 = 2*xmub;
musc_geom.L = L0+.018;
musc_geom.R = L0/2;

step = tsol(2) - tsol(1);
t_ext = tsol(end) :step: 2*apex_t;
tsol_ext = [tsol, t_ext];
t_ext = t_ext - t_ext(1);
y_ext = yfsol(end)+ dyfsol(end).*t_ext-1/2*g.*t_ext.^2;
yfsol_ext = [yfsol; y_ext'];
sphi_ext = sphisol(end) * ones(length(t_ext), 1);
sphisol_ext = [sphisol; sphi_ext];
cphi_ext = cphisol(end) * ones(length(t_ext), 1);
cphisol_ext = [cphisol; cphi_ext];

ff_response.time = tsol_ext';
ff_response.modeswitch = length(tsol);
ff_response.maxiter = length(tsol_ext);
ff_response.sphi = sphisol_ext;
ff_response.cphi = cphisol_ext;
ff_response.yf = yfsol_ext;

ff_geom.Lf = rf;
ff_geom.Lh = rh;
ff_geom.R = 0.0125;
ff_geom.Ls = 0.05;

%% Plotting and Animation

figure(1)
%suptitle('Titin Model Minimum Energy Behavior');
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
plot(tsol, Fmsol/Fmax, 'b');
%plot(tsol, Ftensol/Fmax, 'm');
xlabel('Time [s]'); ylabel('Force [N]');
legend('Activation','Muscle Force (nmlzd)','Location','southeast');
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
        Fsslen(l) = Fmsol(i);
        l=l+1;
    elseif dxmsol(i) < 0
        xsho(s) = xmsol(i);
        Wtsho(s) = Wt(i);
        Lsho(s) = Lsssol(s);
        Fsssho(s) = Fmsol(s);
        s=s+1;
    end
end

figure(2)

subplot(2,1,1);
plot(tsol, asin(sphisol)); hold on;
plot(tsol, cphisol);
plot(tsol, sphisol);
xlabel('Time [s]'); ylabel('Angle [r]');
legend('\phi','cos\phi', 'sin\phi');
grid on;

subplot(2,1,2);
plot(tsol, Fmsol, tsol, Fgsol); hold on;
plot([tsol(1) tsol(end)], [M*g M*g], 'k--');
xlabel('Time [s]'); ylabel('Force [N]');
legend('Fm','Fg');
grid on;


figure(3)
subplot(3,1,1)
plot(tsol, yfsol);
xlabel('Time [s]'); ylabel('Y [m]');
grid on;
subplot(3,1,2)
plot(tsol, dyfsol);
xlabel('Time [s]'); ylabel('Ydot [m/s]');
grid on;
subplot(3,1,3)
plot(tsol, yddotsol);
xlabel('Time [s]'); ylabel('Yddot [m/s2]');
grid on

% ---------------------------------------
%%
%figure(3)

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
%{
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
%}
%%

if (optim.info.status == 0 || optim.info.status == 1) && tsol(end) > 1e-3
    scene1 = Titin_Scene(musc_geom, musc_response);
    Player(scene1);
    
    scene2 = FF_Scene(ff_geom, ff_response);
    Player(scene2);
end

if (optim.info.status == 0 || optim.info.status == 1) && tsol(end) < 1e-3
    disp(['WARNING: Solution is trivial: t = ' num2str(tsol(end)) 's']);
end

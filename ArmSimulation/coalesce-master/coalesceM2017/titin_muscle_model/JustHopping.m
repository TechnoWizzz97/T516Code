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

%% Add States - FrankenFowl
yf = nlp.addState(1, -Inf, Inf, 'Description','State: FF - Y','Length',nNodes);
dyf = nlp.addState(0, -Inf, Inf, 'Description','State: FF - Vel Y','Length',nNodes);

%% Add Inputs
Fm = nlp.addInput(1, 0, Inf, 'Description', 'Input: Muscle Force','Length', nNodes);

%% Add Variables
Fg = nlp.addVariable(1, 0, Inf, 'Description','Variable: GRF','Length',nNodes);
sphi = nlp.addVariable(1, 0, 1, 'Description','Variable: Ankle Angle - sin','Length',nNodes);
cphi = nlp.addVariable(1, 0, 1, 'Description','Variable: Ankle Angle - cos','Length',nNodes);

%% Add Dynamics - Muscle Unit
yfddot = (Fg-Fm-M*g)/M;

%% Add Dynamics - FrankenFowl
nlp.addPdeConstraint(yf, dyf, t, 'Method', IntegrationMethod, 'Description','yf Dynamics');
nlp.addPdeConstraint(dyf, yfddot, t, 'Method', IntegrationMethod, 'Description','dyf Dynamics');

%% Implement Constraints

eps = 1e-5;
eps2 = 1e-4;
eps3 = 1e-3;
eps4 = 1e-2;

% ----- ----- ----- ----- ----- ----- ------ ----- ------ Franken-Fowl ------ ----- ------ ------ ------ ----- ------ ------ ------ ----- ----- -----
% Geometrically constrain yf
nlp.addConstraint(0, yf - rf*sphi, 0, 'Description','Constrain yf geometrically');
nlp.addConstraint(0, rf^2 - ((rf*cphi)^2 + yf^2), 0, 'Description','Define cos phi');

%nlp.addConstraint(0, Fg*(rf/2)*cphi + M*g*(rf/2)*cphi - Fm*(rh+rf/2)*cphi, 0, 'Description','Define GRF - Moment abt forefoot midpoint');  % This breaks ipopt unless its paired with line below - why
nlp.addConstraint(0, Fg*(rf/2) + M*g*(rf/2) - Fm*(rh+rf/2), 0, 'Description','Define GRF - Moment abt forefoot midpoint');  % This breaks ipopt unless its paired with line below - why

% Constrain Force Production
nlp.addConstraint(0, Fg, Inf, 'Description', 'Unilateral Contact Force');
nlp.addConstraint(0, Fm, Inf, 'Description', 'No ss compression');

% Constrain y to be > 0
nlp.addConstraint(0, yf, Inf, 'Description','Keep above ground');

% Initial Contidion - FrankenFowl
%nlp.addConstraint(0, yf.initial, rf/8, 'Description','Initial Condition: yf');
nlp.addConstraint(0, dyf.initial, 0, 'Description','Initial Condition: dyf');
nlp.addConstraint(M*g, Fg.initial, M*g, 'Description','Initial Conditions: Fg');



h = 0.01;
tapex = dyf.final/g;

% Termination Conditions
nlp.addConstraint(0, dyf.final, Inf, 'Description','Termination Condition: Takeoff - Velocity');
nlp.addConstraint(0, 2*g*h - dyf.final^2, 0, 'Description','Termination Condition: Takeoff - Projectile Motion');
nlp.addConstraint(0, Fg.final, 0, 'Description','Termination Condition: Takeoff - Force');
nlp.addConstraint(eps, t.final, 1, 'Description','Termination Condition: Time');

%% Define Objective and Solve
nlp.addObjective(trapz(Fm^2, t) - yf.final, 'Description','Minimize CE force^2');
%nlp.addObjective(trapz(Fg, t), 'Description', 'Minimize GRF');
%nlp.addObjective(0, 'Description', 'No objective. Solving dynamics');
%nlp.addObjective(t,'Description','Minimize Time');

optim = Ipopt(nlp);
optim.options.ipopt.max_iter = 5000;
optim.export;
optim.solve;

%%

tsol = linspace(0, squeeze(eval(t)), nNodes);
Fmsol = squeeze(eval(Fm));
yddotsol = squeeze(eval(yfddot));
yfsol = squeeze(eval(yf));
dyfsol = squeeze(eval(dyf));
sphisol = squeeze(eval(sphi));
cphisol = squeeze(eval(cphi));
Fgsol = squeeze(eval(Fg));
apex_t = squeeze(eval(tapex));

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
    scene2 = FF_Scene(ff_geom, ff_response);
    Player(scene2);
end

if (optim.info.status == 0 || optim.info.status == 1) && tsol(end) < 1e-3
    disp(['WARNING: Solution is trivial: t = ' num2str(tsol(end)) 's']);
end

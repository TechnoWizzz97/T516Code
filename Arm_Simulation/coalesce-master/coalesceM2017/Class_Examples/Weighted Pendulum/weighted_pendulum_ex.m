clc;
clear all;
close all;

mb = 1;                      % [kg] mass of ball
ma = 0.5;                   % [kg] mass of arm
r = 0.5;                       % [m] Lengh of arm
g = 9.81;                    % [m/s2] Gravitational constant
Ib = mb*r^2;             % [kg*m2] Moment of ball abt center of rotation
Ia = 1/3*ma*r^2;      % [kg*m2] Moment of arm abt center of rotation
b = 0.05;                    % rotational damping
d = 0.1;                      % [m] Weight Diameter

I = Ia+Ib;

nNodes = 101;

nlp = DirectCollocation(nNodes); 

% Add Time
t = nlp.addTime;

% Add States
th = nlp.addState(0,-Inf, Inf, 'Description','State: Theta','Length',nNodes);
thdot = nlp.addState(0,-Inf, Inf, 'Description','State: Theta dot','Length',nNodes);

% Add Variable
tor_abs = nlp.addVariable(0, -Inf, Inf, 'Description','Variable: abs_tor','Length',nNodes);

% Add Inputs
tor = nlp.addInput(0, -Inf, Inf, 'Description', 'Input: Torque','Length',nNodes);

% Add dynamics
nlp.addPdeConstraint(th, thdot, t, 'Method', 'Explicit Euler','Description','thdot dynamics');
nlp.addPdeConstraint(thdot, r*g/I*(-mb*cos(th)-ma*cos(th)/2)+tor/I -b*thdot, t, 'Method', 'Explicit Euler','Description','alpha dynamics');

nlp.addConstraint(-10, tor, 10); % Torque Limit
nlp.addConstraint(0, th.initial, 0); % Initial Position
nlp.addConstraint(0, thdot.initial, 0); % Initial Velocity
nlp.addConstraint(pi/2, th.final, pi/2); % Final Position
nlp.addConstraint(0, thdot.final, 0); % Final Velocity

nlp.addConstraint(0, tor + tor_abs, Inf); % Absolute value constraint
nlp.addConstraint(0, tor_abs - tor, Inf); % Absolute value constraint
nlp.addConstraint(0, t.final, 1); % Constrain time to 1 second

nlp.addObjective(trapz(tor_abs, t), 'Description','Minimize Integral torque^2');

optim = Ipopt(nlp);
optim.export;
optim.solve;


thsol = squeeze(eval(th));
thdotsol = squeeze(eval(thdot));
torsol = squeeze(eval(tor));
tsol = linspace(0, squeeze(eval(t)), nNodes);

subplot(3,1,1)
plot(tsol, thsol)
grid on;
ylabel('\Theta [rad]');
subplot(3,1,2);
plot(tsol, thdotsol);
grid on;
ylabel('\Thetadot [rad]');
subplot(3,1,3);
plot(tsol,torsol);
grid on;
ylabel('\tau [Ns]');

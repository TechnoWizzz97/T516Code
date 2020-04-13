%MODEL ATRIAS planar model formulation.
%
% Description:
%   Derives equations of motion for the 2D planar ATRIAS model in single
%   support. Leg mass is ignored.
%
% Copyright 2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Time
syms t;

% Generalized coordinates
px = sym('px(t)');
pz = sym('pz(t)');
qt = sym('qt(t)');
qLlA = sym('qLlA(t)');
qLlB = sym('qLlB(t)');
qLmA = sym('qLmA(t)');
qLmB = sym('qLmB(t)');
qRlA = sym('qRlA(t)');
qRlB = sym('qRlB(t)');
qRmA = sym('qRmA(t)');
qRmB = sym('qRmB(t)');

% Control inputs
tauLmA = sym('tauLmA(t)');
tauLmB = sym('tauLmB(t)');
tauRmA = sym('tauRmA(t)');
tauRmB = sym('tauRmB(t)');

% Environmental parameters
g = 9.81;

% Lower back leg segment (wrt knee)
I1com = 0.0147; I1 = 0.0236; m1 = 0.4556; l1com = 0.1394; l1 = 0.5;

% Upper back leg segment (wrt hip)
I2com = 0.0227; I2 = 0.0434; m2 = 0.6882; l2com = 0.1705; l2 = 0.5;

% Lower front leg segment (wrt ankle)
I3com = 0.0142; I3 = 0.0781; m3 = 0.4786; l3com = 0.365; l3 = 0.5;

% Upper front leg segment (wrt hip)
I4com = 0.0182; I4 = 0.0348; m4 = 0.7477; l4com = 0.146; l4 = 0.4;

% Harmonic drive gear ratio
G = 50;

% Torso
Itcom = 1.60427586; It = 4.09750513; mt = 22.22268345; ltcom = 0.33493699;

% Drive housing
Idcom = 0.2616; Id = 0.2759; md = 11.6828; ldcom = 0.0349;

% Actuator rotor
Iacom = 0.00121/2; Ia = 0.00121/2; ma = 1.8144/2; ba = 19; lacom = 0;
ba = sym('ba');

% Actuator output
Iocom = 0.0267; Io = 0.041; mo = 1.9683; locom = 0;

% Springs
ks = sym('ks'); bs = 1.49;

% Initialize
U = sym(0); T = sym(0); D = sym(0); Q = sym(0);

% Torso
x = px + ltcom*cos(qt);
z = pz + ltcom*sin(qt);
U = U + mt*g*z;
T = T + 1/2*mt*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*(Itcom+2*Idcom)*diff(qt, t)^2;

% Actuators
x = px;
z = pz;
U = U + (4*ma+4*mo+2*md)*g*z;
T = T + 1/2*(4*ma+4*mo+2*md)*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*(Iocom+Iacom*G^2)*diff(qLmA, t)^2 + 1/2*(Iocom+Iacom*G^2)*diff(qLmB, t)^2 + ...
	1/2*(Iocom+Iacom*G^2)*diff(qRmA, t)^2 + 1/2*(Iocom+Iacom*G^2)*diff(qRmB, t)^2;
D = D + 1/2*ba*diff(qLmA - qt, t)^2 + 1/2*ba*diff(qLmB - qt, t)^2 + ...
	1/2*ba*diff(qRmA - qt, t)^2 + 1/2*ba*diff(qRmB - qt, t)^2;
Q = Q + tauLmA*(qLmA - qt) + tauLmB*(qLmB - qt) + ...
	tauRmA*(qRmA - qt) + tauRmB*(qRmB - qt);

% Springs
U = U + 1/2*ks*(qLmA - qLlA)^2 + 1/2*ks*(qLmB - qLlB)^2 + ...
	1/2*ks*(qRmA - qRlA)^2 + 1/2*ks*(qRmB - qRlB)^2;
D = D + 1/2*bs*diff(qLmA - qLlA, t)^2 + 1/2*bs*diff(qLmB - qLlB, t)^2 + ...
	1/2*bs*diff(qRmA - qRlA, t)^2 + 1/2*bs*diff(qRmB - qRlB, t)^2;

% Left (Stance) upper back leg segment
x = px + l2com*cos(qLlB);
z = pz + l2com*sin(qLlB);
U = U + m2*g*z;
T = T + 1/2*m2*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I2com*diff(qLlB, t)^2;

% Left (Stance) lower back leg segment
x = px + l2*cos(qLlB) + l1com*cos(qLlA);
z = pz + l2*sin(qLlB) + l1com*sin(qLlA);
U = U + m1*g*z;
T = T + 1/2*m1*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I1com*diff(qLlA, t)^2;

% Left (Stance) upper front leg segment
x = px + l4com*cos(qLlA);
z = pz + l4com*sin(qLlA);
U = U + m4*g*z;
T = T + 1/2*m4*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I4com*diff(qLlA, t)^2;

% Left (Stance) lower front leg segment
x = px + l4*cos(qLlA) + (l3 - l3com)*cos(qLlB);
z = pz + l4*sin(qLlA) + (l3 - l3com)*sin(qLlB);
U = U + m3*g*z;
T = T + 1/2*m3*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I3com*diff(qLlB, t)^2;

% Right (Stance) upper back leg segment
x = px + l2com*cos(qRlB);
z = pz + l2com*sin(qRlB);
U = U + m2*g*z;
T = T + 1/2*m2*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I2com*diff(qRlB, t)^2;

% Right (Stance) lower back leg segment
x = px + l2*cos(qRlB) + l1com*cos(qRlA);
z = pz + l2*sin(qRlB) + l1com*sin(qRlA);
U = U + m1*g*z;
T = T + 1/2*m1*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I1com*diff(qRlA, t)^2;

% Right (Stance) upper front leg segment
x = px + l4com*cos(qRlA);
z = pz + l4com*sin(qRlA);
U = U + m4*g*z;
T = T + 1/2*m4*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I4com*diff(qRlA, t)^2;

% Right (Stance) lower front leg segment
x = px + l4*cos(qRlA) + (l3 - l3com)*cos(qRlB);
z = pz + l4*sin(qRlA) + (l3 - l3com)*sin(qRlB);
U = U + m3*g*z;
T = T + 1/2*m3*(diff(x, t)^2 + diff(z, t)^2) + ...
	1/2*I3com*diff(qRlB, t)^2;

% Define system Lagrangian
L = T - U;

% Define generalized coordinate, input, and parameters arrays
q = sym('[px pz qt qLlA qLlB qLmA qLmB qRlA qRlB qRmA qRmB]');
u = sym('[tauLmA tauLmB tauRmA tauRmB]');
p = sym('[ks bs]');

% Compute equations of motion from Lagrangian
[M, f, x] = lagrangian(L, D, Q, q, u, p);

% Write out simple matlab code for expressions
sym2matlab(M,'M');
sym2matlab(f,'f');

% Declare symbolics without time dependance
px = sym('px');
pz = sym('pz');
qt = sym('qt');
qLlA = sym('qLlA');
qLlB = sym('qLlB');
qLmA = sym('qLmA');
qLmB = sym('qLmB');
qRlA = sym('qRlA');
qRlB = sym('qRlB');
qRmA = sym('qRmA');
qRmB = sym('qRmB');

% Positions
x_torso = px + ltcom*cos(qt);
z_torso = pz + ltcom*sin(qt);

% Feet
x_Lfoot = px + l2*cos(qLlB) + l1*cos(qLlA);
z_Lfoot = pz + l2*sin(qLlB) + l1*sin(qLlA);
x_Rfoot = px + l2*cos(qRlB) + l1*cos(qRlA);
z_Rfoot = pz + l2*sin(qRlB) + l1*sin(qRlA);


% SS Constraints
g_ss = [x_Lfoot; z_Lfoot];
G_ss = jacobian(g_ss, x(1:11));
Gq_ss = jacobian(G_ss*x(12:22), x(1:11))*x(12:22);
sym2func(g_ss, {x});
sym2func(G_ss, {x});
sym2func(Gq_ss, {x});

% DS Constraints
g_ds = [x_Lfoot; z_Lfoot; x_Rfoot; z_Rfoot];
G_ds = jacobian(g_ds, x(1:11));
Gq_ds = jacobian(G_ds*x(12:22), x(1:11))*x(12:22);
sym2func(g_ds, {x});
sym2func(G_ds, {x});
sym2func(Gq_ds, {x});

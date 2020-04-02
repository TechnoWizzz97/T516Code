%RUNWALK Runs the walking optimization on Durus system.
%
% Copyright 2013-2014 Mikhail S. Jones

% Add function expression folder to path
clear all; addpath('dev');

% Construct system model
model = DurusSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = model.directCollocation(100, ...
	'modeSchedule', {'ss'}, ...
	'periodic', true);

% Change initial guess for duration
t.initialGuess = 0.5;

% Add objective
d = nlp.addVariable(0, 0.5, Inf);
D = pe_com_vec(x.final) - pe_com_vec(x.initial);
nlp.addConstraint(0, d - D(1), 0);
nlp.addConstraint(0.25, t, Inf);
nlp.addObjective(trapz(u.'*u,t)/(9.81*80*d));

% Joint angles
nlp.addConstraint(-pi/2, x(3), pi/2);
nlp.addConstraint(-pi/2, x(5), pi/2);
nlp.addConstraint(0, x(6), pi/2);
nlp.addConstraint(-pi/2, x(7), pi/2);
nlp.addConstraint(-pi/4, x(8), pi/4);
nlp.addConstraint(0, x(9), pi/2);
nlp.addConstraint(-pi/2, x(10), pi/2);

% Relabeling matrix
R = [ 1 0 0 0 0 0 0 0 0 0 0;
			0 1 0 0 0 0 0 0 0 0 0;
			0 0 1 0 1 1 1 -1 -1 -1 0;
			0 0 0 0 0 0 0 0 0 0 -1;
			0 0 0 0 0 0 0 0 0 1 0;
			0 0 0 0 0 0 0 0 1 0 0;
			0 0 0 0 0 0 0 1 0 0 0;
			0 0 0 0 0 0 1 0 0 0 0;
			0 0 0 0 0 1 0 0 0 0 0;
			0 0 0 0 1 0 0 0 0 0 0;
			0 0 0 -1 0 0 0 0 0 0 0];

% Impact map
D = De_mat(x.final);
G = Jh_nsf_mat(x.final);
n = size(G, 1);
M = [D, -G.'; G, zeros(n)];
f = [D*x(12:22).final; zeros(n,1)];
Fe = nlp.addVariable(zeros(n,1), -Inf(n,1), Inf(n,1), ...
		'Description', 'Impact Forces');
dxe = nlp.addVariable(zeros(11,1), -Inf(11,1), Inf(11,1));
nlp.addConstraint(0, M*[dxe; Fe] - f, 0);

% Add periodic orbit constraints
nlp.addConstraint(0, R*x(1:11).final - x(1:11).initial, 0);
nlp.addConstraint(0, R*dxe - x(12:22).initial, 0);

% Impact forces
nlp.addConstraint(0, Fe(2), Inf); % Positive vertical force
nlp.addConstraint(0, Fe(2) - Fe(1), Inf); % Friction cone
nlp.addConstraint(0, Fe(2) + Fe(1), Inf); % Friction cone

% Impact velocities
% TODO

% Start stance foot at origin
pe_sf = pe_sf_vec(x.initial);
nlp.addConstraint(0, pe_sf, 0);

% Non stance foot ground clearance
% toeProfile = 0.025*sin(pi*(0:nlp.nNodes - 1)/(nlp.nNodes - 1));
pe_nsf = pe_nsf_vec(x);
nlp.addConstraint(0, pe_nsf(2), Inf);

% Stance foot force (non-negative)
% F = [nlp.getVariable('Lagrange Multiplier').expression];
% nlp.addConstraint(0, F(2), Inf); % Positive vertical force
% nlp.addConstraint(0, F(2) - F(1), Inf); % Friction cone
% nlp.addConstraint(0, F(2) + F(1), Inf); % Friction cone
% nlp.addConstraint(0, F(2).final, 0); % Zero force on TO

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;
%
% % Construct time response object
% response = nlp.getResponse;
%
% % Plot time response
% nlp.plot;
%
% % Construct and play animation
% scene = DurusScene(model, response);
% Player(scene);
%
% figure;
% plot(nlp.time.solution, vertcat(nlp.getVariable('Lagrange Multiplier').solution)');
%
%
% absw1 = abs(nlp.stateVariable(5+11).solution.*nlp.inputVariable(1).solution);
% absw2 = abs(nlp.stateVariable(6+11).solution.*nlp.inputVariable(2).solution);
% absw3 = abs(nlp.stateVariable(7+11).solution.*nlp.inputVariable(3).solution);
% absw4 = abs(nlp.stateVariable(8+11).solution.*nlp.inputVariable(4).solution);
% absw5 = abs(nlp.stateVariable(9+11).solution.*nlp.inputVariable(5).solution);
% absw6 = abs(nlp.stateVariable(10+11).solution.*nlp.inputVariable(6).solution);
% absw = absw1 + absw2 + absw3 + absw4 + absw5 + absw6;
% W = trapz(nlp.time.solution, absw);
% pe_com = pe_com_vec(response.states{1}(end,:)) - pe_com_vec(response.states{1}(1,:));
% d = pe_com(1);
% mcot = W/(9.81*80*d)
%
% nlp.getVariable('Impact Forces').solution

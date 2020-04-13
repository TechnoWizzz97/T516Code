%RUNWALK Runs the walking optimization on ATRIAS system.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up workspace
clear all;

% Construct system model
sys = AtriasSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u, F] = sys.directCollocation(200, ...
	'modeSchedule', {'ss'}, ...
	'periodic', true);

% Set bounds
t.lowerBound = 0.5;

% Set initial guesses
t.initialGuess = 0.5;
x(2).initialGuess = 1;
x(3).initialGuess = pi/2;
x(4).initialGuess = -pi/2 - 0.2;
x(6).initialGuess = -pi/2 - 0.2;
x(8).initialGuess = -pi/2 - 0.2;
x(10).initialGuess = -pi/2 - 0.2;
x(5).initialGuess = -pi/2 + 0.2;
x(7).initialGuess = -pi/2 + 0.2;
x(9).initialGuess = -pi/2 + 0.2;
x(11).initialGuess = -pi/2 + 0.2;

% Add objective
d = nlp.addVariable(0.5, 0.1, Inf);
nlp.addConstraint(0, d - (x(1).final - x(1).initial), 0);
nlp.addObjective(trapz(u'*u, t)/(9.81*60*d));

% Joint hard stop limits
lExtLim = deg2rad(34.5);
lFleLim = deg2rad(160);
lAMin = deg2rad(72.5);
lAMax = deg2rad(225);
lBMin = deg2rad(135);
lBMax = deg2rad(287.5);

% Joint angles
nlp.addConstraint(pi/4, x(3), 3*pi/4);
nlp.addConstraint(lExtLim, x(7) - x(6), lFleLim);
nlp.addConstraint(lAMin, x(3) - x(6), lAMax);
nlp.addConstraint(lBMin, x(3) - x(7), lBMax);
nlp.addConstraint(lExtLim, x(11) - x(10), lFleLim);
nlp.addConstraint(lAMin, x(3) - x(10), lAMax);
nlp.addConstraint(lBMin, x(3) - x(11), lBMax);

% Relabeling matrix
R = [	1 0 0 0 0 0 0 0 0 0 0;
			0 1 0 0 0 0 0 0 0 0 0;
			0 0 1 0 0 0 0 0 0 0 0;
			0 0 0 0 0 0 0 1 0 0 0;
			0 0 0 0 0 0 0 0 1 0 0;
			0 0 0 0 0 0 0 0 0 1 0;
			0 0 0 0 0 0 0 0 0 0 1;
			0 0 0 1 0 0 0 0 0 0 0;
			0 0 0 0 1 0 0 0 0 0 0;
			0 0 0 0 0 1 0 0 0 0 0;
			0 0 0 0 0 0 1 0 0 0 0];

% Impact map
[D, ~] = sys.secondOrderStateEquation(0, x(1:11).final, x(12:22).final, u.final, '');
[g, G, Gq] = sys.constraintEquation(R*x(1:11).final, R*x(12:22).final, 'ss');
G = G*R;
n = size(G, 1);
M = [D, -G.'; G, zeros(n)];
f = [D*x(12:22).final; zeros(n,1)];
dxe = nlp.addVariable(zeros(11,1), -Inf(11,1), Inf(11,1));
Fe = nlp.addVariable(zeros(n,1), -Inf(n,1), Inf(n,1), ...
	'Description', 'Impact Forces');
nlp.addConstraint(0, M*[dxe; Fe] - f, 0);

% Add periodic orbit constraints
p = R*x(1:11).final - x(1:11).initial;
nlp.addConstraint(0, p(2:11), 0); % match everything except x position
nlp.addConstraint(0, R*dxe - x(12:22).initial, 0);

% Impact forces
nlp.addConstraint(0, Fe(2), Inf); % Positive vertical force
nlp.addConstraint(0, Fe(2) - Fe(1), Inf); % Friction cone
nlp.addConstraint(0, Fe(2) + Fe(1), Inf); % Friction cone

% Impact velocities (TO toe must have positive velocity)
[g, G, Gq] = sys.constraintEquation(x(1:11).initial, x(12:22).initial, 'ss');
nlp.addConstraint(0, G(2,:)*x(12:22).initial, Inf);

% Start stance foot at origin
[g, G, Gq] = sys.constraintEquation(x(1:11).initial, x(12:22).initial, 'ds');
nlp.addConstraint(0, g(1:2), 0);

% Non stance foot ground clearance
toeProfile = 0.1*sin(pi*(0:nlp.nNodes - 1)/(nlp.nNodes - 1));
[g, G, Gq] = sys.constraintEquation(x(1:11), x(12:22), 'ds');
nlp.addConstraint(toeProfile, g(4,1), Inf);

% Stance foot force (non-negative)
nlp.addConstraint(0, F(2), Inf); % Positive vertical force
nlp.addConstraint(0, F(2) - F(1), Inf); % Friction cone
nlp.addConstraint(0, F(2) + F(1), Inf); % Friction cone

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
tic;
optim.solve;
toc;

% Construct time response object
response = nlp.getResponse;

% Plot time response
nlp.plot;

% Construct and play animation
scene = AtriasScene(sys, response);
Player(scene);

figure;
plot(squeeze(eval(F))')

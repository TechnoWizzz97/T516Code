%RUNSWINGUP Finds a time optimal swing-up maneuver for the acrobot.
%
% Description:
%   A time optimal swing-up maneuver is found for an acrobot using
%		trajectory optimization.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AcrobotSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Add minimum time objective
nlp.addObjective(t);

% Set bounds (Leave a little overhead for control)
u.lowerBound = -0.8*sys.tauLim;
u.upperBound = 0.8*sys.tauLim;

% Set initial guess
t.initialGuess = 2;
x(1).initialGuess = -pi/2;
x(2).initialGuess = -pi/2;

% Add initial condition constraints
nlp.addConstraint(-pi/2, x(1:2).initial, -pi/2);
nlp.addConstraint(0, x(3:4).initial, 0);

% Add final condition constraints
nlp.addConstraint(pi/2, x(1:2).final, pi/2);
nlp.addConstraint(0, x(3:4).final, 0);
nlp.addConstraint(0, u.final, 0);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Construct time response object
response = nlp.getResponse;

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

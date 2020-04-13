%RUNSWING Finds a energy optimal swinging maneuver for the acrobot.
%
% Description:
%   A energy optimal swinging maneuver is found for an acrobot using
%		trajectory optimization.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AcrobotSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Add minimum energy objective
nlp.addObjective(trapz(u^2, t));

% Set bounds
t.lowerBound = 1;

% Set initial guess
x(1).initialGuess = -pi/2;
x(2).initialGuess = -pi/2;

% Add initial condition constraints
nlp.addConstraint(0, x.initial, 0);
nlp.addConstraint(0, u.initial, 0);

% Add final condition constraints
nlp.addConstraint(0, x.final, 0);
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

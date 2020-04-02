%RUNENERGYOPTIMAL Runs energy optimal quadrotor maneuver problem.
%
% Description:
%   A quadrotor must take off, perform a flip, and land at the target
%   position using the least amount of energy possible.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = QuadrotorSystem;

% Construct trajectory optimization problem
[nlp, t, x, u] = sys.directCollocation(200);

% Set bounds
t.lowerBound = 1;

% Set initial guesses
t.initialGuess = 2;

% Add minimum energy objective
nlp.addObjective(trapz(u.'*u, t));

% Add ground clearance constraint
nlp.addConstraint(0, x(2) - sys.L/2*sin(x(3)), Inf);
nlp.addConstraint(0, x(2) + sys.L/2*sin(x(3)), Inf);

% Add initial condition constraints
nlp.addConstraint(0, x.initial, 0);

% Add final condition constraints
nlp.addConstraint(10, x(1).final, 10);
nlp.addConstraint(0, x(2).final, 0);
nlp.addConstraint(2*pi, x(3).final, 2*pi);
nlp.addConstraint(0, x(4:6).final, 0);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Get time response
response = nlp.getResponse;

% Plot response
response.plot;

% Construct and play animation
scene = QuadrotorScene(sys, response);
Player(scene);

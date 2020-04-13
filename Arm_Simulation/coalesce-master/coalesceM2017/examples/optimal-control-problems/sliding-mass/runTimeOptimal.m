%RUNTIMEOPTIMAL Runs time optimal sliding mass problem.
%
% Description:
%   A sliding mass must move from its current state to a target state in
%		the minimum amount of time possible.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = SlidingMassSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Set bounds
u.lowerBound = -1;
u.upperBound = 1;

% Add minimum time objective
nlp.addObjective(t);

% Add initial condition constraints
nlp.addConstraint(0, x.initial, 0);

% Add final condition constraints
nlp.addConstraint(1, x(1).final, 1);
nlp.addConstraint(0, x(2).final, 0);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Get time response
response = nlp.getResponse;

% Plot response
response.plot;

% Construct and play animation
scene = SlidingMassScene(sys, response);
Player(scene);

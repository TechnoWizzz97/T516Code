%RUNENERGYOPTIMAL Runs energy optimal sliding mass problem.
%
% Description:
%   A sliding mass must reverse its velocity in a fixed amount of time
%   using the minimum amount of energy possible.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = SlidingMassSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Set bounds
t.lowerBound = 1;
t.upperBound = 1;
x(1).upperBound = 1/9;

% Add minimum energy objective
nlp.addObjective(trapz(u^2, t)/2);

% Add initial condition constraints
nlp.addConstraint(0, x(1).initial, 0);
nlp.addConstraint(1, x(2).initial, 1);

% Add final condition constraints
nlp.addConstraint(0, x(1).final, 0);
nlp.addConstraint(-1, x(2).final, -1);

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

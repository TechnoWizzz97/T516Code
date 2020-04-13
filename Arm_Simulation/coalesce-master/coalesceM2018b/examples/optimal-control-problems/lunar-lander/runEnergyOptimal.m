%RUNENERGYOPTIMAL Runs energy optimal lunar lander problem.
%
% Description:
%   A lunar lander must move from its current state to a target state using
%   the minimum amount of energy possible.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = LunarLanderSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Add minimum energy objective
nlp.addObjective(trapz(u, t));

% Add initial condition constraints
nlp.addConstraint(10, x(1).initial, 10);
nlp.addConstraint(-2, x(2).initial, -2);

% Add final condition constraints
nlp.addConstraint(0, x.final, 0);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Plot
nlp.plot;

% Get time response
response = nlp.getResponse;

% Construct and play animation
scene = LunarLanderScene(sys, response);
Player(scene);

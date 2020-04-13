%RUNENERGYOPTIMAL Run energy optimalVan der Pol oscillator stabilization.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = VanDerPolSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Set bounds
u.lowerBound = -Inf;
u.upperBound = 0.75;
t.lowerBound = 10;
t.upperBound = 10;

% Add minimum energy objective
nlp.addObjective(trapz(x.'*x + u^2, t));

% Add initial condition constraints
nlp.addConstraint(1, x(1).initial, 1);
nlp.addConstraint(0, x(2).initial, 0);

% Add final condition constraints
nlp.addConstraint(0, x.final, 0);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Plot
nlp.plot;

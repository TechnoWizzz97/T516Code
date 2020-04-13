%RUNTIMEOPTIMAL Runs time optimal brachistochrone problem.
%
% Description:
%   Find the curve or path that moves a point mass from one location to
%		another in the least amount of time possible. An example would be a
%		bead on a wire released from rest.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = BrachistochroneSystem;

% Construct trajectory optimization phase object
[nlp, t, x, u] = sys.directCollocation(200);

% Add minimum time objective
nlp.addObjective(t);

% Add initial condition constraints
nlp.addConstraint(0, x(1:3).initial, 0);

% Add final condition constraints
nlp.addConstraint(2, x(1:2).final, 2);

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Plot response
nlp.plot;

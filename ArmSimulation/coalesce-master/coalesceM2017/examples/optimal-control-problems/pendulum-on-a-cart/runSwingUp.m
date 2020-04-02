function [sys, response] = runSwingUp(nLinks)
%RUNSWINGUP Runs n-link pendulum on a cart swing-up problem.
%
% Description:
%   A n-link pendulum on a cart must perform a swing-up maneuver in the
%		least amount of time possible.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct system model
	sys = PendulumCartSystem(nLinks);

	% Construct trajectory optimization phase object
	[nlp, t, x, u] = sys.directCollocation(200);

	% Index of link positions and velocities
	xi = (1:nLinks) + 1;
	dxi = (1:nLinks) + 2 + nLinks;

	% Set bounds
	u.lowerBound = -0.8*sys.fLim;
	u.upperBound = 0.8*sys.fLim;
	x(1).lowerBound = -sys.xLim;
	x(1).upperBound = sys.xLim;

	% Set initial guess
	t.initialGuess = 2;
	for i = 1:numel(xi)
		x(xi(i)).initialGuess = -pi/2;
	end % for

	% Add minimum time objective
	nlp.addObjective(t);

	% Add initial condition constraints
	nlp.addConstraint(0, x(1).initial, 0);
	nlp.addConstraint(0, x(nLinks+2).initial, 0);
	nlp.addConstraint(-pi/2, x(xi).initial, -pi/2);
	nlp.addConstraint(0, x(dxi).initial, 0);
	nlp.addConstraint(0, u.initial, 0);

	% Add final condition constraints
	nlp.addConstraint(0, x(1).final, 0);
	nlp.addConstraint(0, x(nLinks+2).final, 0);
	nlp.addConstraint(pi/2, x(xi).final, pi/2);
	nlp.addConstraint(0, x(dxi).final, 0);
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
	scene = PendulumCartScene(sys, response);
	Player(scene);
end % runSwingUp

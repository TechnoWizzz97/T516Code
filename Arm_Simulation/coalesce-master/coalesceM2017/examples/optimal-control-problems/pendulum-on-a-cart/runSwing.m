function [sys, response] = runSwing(nLinks)
%RUNSWING Runs n-link pendulum on a cart swinging problem.
%
% Description:
%   A n-link pendulum on a cart must perform a periodic swinging maneuver
%		consuming the least amount of energy as possible.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct system model
	sys = PendulumCartSystem(nLinks);

	% Construct trajectory optimization phase object
	[nlp, t, x, u] = sys.directCollocation(200);

	% Set bounds
	x(1).lowerBound = -sys.xLim;
	x(1).upperBound = sys.xLim;

	% Set initial guess
	t.initialGuess = 2;
	for i = (1:nLinks) + 1
		x(i).initialGuess = -pi/2;
	end % for

	% Add minimum time objective
	nlp.addObjective(trapz(u^2, t));

	% Add initial condition constraints
	nlp.addConstraint(0, x.initial, 0);

	% Add final condition constraints
	nlp.addConstraint(0, x.final, 0);

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
end % runSwing

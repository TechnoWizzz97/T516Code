function [sys, response] = runPassive(nLinks)
%RUNPASSIVE Runs passive n-link pendulum on a cart simulation.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct system model
	sys = PendulumCartSystem(nLinks);

	% Construct initial conditions
	x0 = [0 repmat(pi/2, 1, nLinks) 0 repmat(0, 1, nLinks)];

	% Simulate passive system
	response = sys.simulate([0 10], x0);

	% Plot time response
	response.plot;

	% Construct and play animation
	scene = PendulumCartScene(sys, response);
	Player(scene);
end % runPassive

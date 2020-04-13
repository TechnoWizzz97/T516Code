function runFixedPointTiLqr(nLinks)
%RUNFIXEDPOINTTILQR Runs LTI LQR n-link pendulum on a cart simulation.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct system model
	sys = PendulumCartSystem(nLinks);

	% Derive fixed point
	xGuess = [0 repmat(pi/2, 1, nLinks) 0 repmat(0, 1, nLinks)]';
	uGuess = 0;
	point = sys.fixedPoint(xGuess, uGuess);

	% Derive time invariant LQR controller
	controller = sys.tilqr(point);

	% Simulate fixed-point balance from perturbed initial conditions
	x0 = point.states + randn(2 + 2*nLinks, 1)/(10*nLinks^2);
	response = sys.simulate([0 5], x0, ...
		'Controller', controller);

	% Plot time response
	response.plot;

	% Construct and play animation
	scene = PendulumCartScene(sys, response);
	Player(scene);
end % runFixedPointTiLqr

function [sys, response] = runSwingUpTvLqr(nLinks)
%RUNSWINGUPTVLQR Runs n-link pendulum on a cart swing-up problem.
%
% Description:
%   A n-link pendulum on a cart must perform a swing-up maneuver in the
%		least amount of time possible. The trajectory is then stabilized using
%		a time varying linear quadratic regulator
%
% Copyright 2013-2014 Mikhail S. Jones

	% Run swing up script
	[sys, response] = runSwingUp(nLinks);

	% Derive time-variant LQR controller
	controller = sys.tvlqr(response);

	% Simulate swing-up and balance from perturbed initial conditions
	x0 = response.states{1}(:,1);
	response = sys.simulate([0 5], x0, ...
		'Controller', controller);

	% Plot time response
	response.plot;

	% Construct and play animation
	scene = PendulumCartScene(sys, response);
	Player(scene);
end % runSwingUpTvLqr

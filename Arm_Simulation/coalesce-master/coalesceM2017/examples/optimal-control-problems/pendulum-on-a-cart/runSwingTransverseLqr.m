function [sys, response] = runSwingTransverseLqr(nLinks)
%RUNSWINGTRANSVERSELQR Stabilizes a swinging maneuver with transverse LQR.
%
% Description:
%   A transverse linear quadratic regulator is designed around an energy
%		optimal periodic swinging maneuver for a n-link pendulum on a cart. The
%	  swinging maneuver is found using a direct collocation trajectory
%		optimization.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Run swing script
	[sys, response] = runSwing(nLinks);

	% Derive transverse LQR controller
	controller = sys.transverselqr(response);

	% Simulate swinging from perturbed initial conditions
	x0 = response.states{1}(:,1);
	response = sys.simulate([0 10], x0, ...
		'Controller', controller);

	% Plot time response
	response.plot;

	% Construct and play animation
	scene = PendulumCartScene(sys, response);
	Player(scene);
end % runSwingTransverseLqr

%RUNSWINGUPTVLQR Stabilizes a swing-up maneuver with time varying LQR.
%
% Description:
%   A time varying linear quadratic regulator is designed around a time
%		optimal swing-up maneuver for the acrobot. The swing-up maneuver is
%		found using a direct collocation trajectory optimization.
%
% Copyright 2013-2014 Mikhail S. Jones

% Run swing up script
runSwingUp;

% Derive time-variant LQR controller
controller = sys.tvlqr(response, 'R', 0.1);

% Simulate swing-up and balance from perturbed initial conditions
response = sys.simulate([0 5], [-pi/2 -pi/2 0 0], ...
	'Controller', controller);

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

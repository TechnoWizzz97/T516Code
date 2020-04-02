%RUNSWINGTRANSVERSELQR Stabilizes a swinging maneuver with transverse LQR.
%
% Description:
%   A transverse linear quadratic regulator is designed around an energy
%		optimal periodic swinging maneuver for the acrobot. The swinging
%		maneuver is found using a direct collocation trajectory optimization.
%
% Copyright 2013-2014 Mikhail S. Jones

% Run swing script
runSwing;

% Derive transverse LQR controller
controller = sys.transverselqr(response);

% Simulate swinging from perturbed initial conditions
response = sys.simulate([0 10], [-pi -pi 0 0], ...
	'Controller', controller);

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

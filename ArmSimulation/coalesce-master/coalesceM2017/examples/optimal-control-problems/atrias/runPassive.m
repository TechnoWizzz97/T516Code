%RUNPASSIVE Runs the passive ATRIAS system.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AtriasSystem;

% Initial posture
l = 0.9;
qA = - pi/2 + acos(l);
qB = - pi/2 - acos(l);

% Initial posture
x0 = [0; l; pi/2; qA+0.2; qB+0.2; qA+0.2; qB+0.2; qA-0.2; qB-0.2; qA-0.2; qB-0.2; zeros(11,1)];

% Simulate passive system
response = sys.simulate([0 5], x0, ...
	'initialMode', 'ss');

% Plot time response
response.plot;

% Construct and play animation
scene = AtriasScene(sys, response);
Player(scene);

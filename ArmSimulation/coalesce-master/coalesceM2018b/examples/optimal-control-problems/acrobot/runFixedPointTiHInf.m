%RUNFIXEDPOINTTIHINF Stabilizes a fixed point on the acrobot using H-Inf.
%
% Description:
%   A time invariant H-Infinity controller is designed around the
%		unstable fixed point for the acrobot. The controller is used to
%		stabilize the system from initial condition perturbations.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AcrobotSystem;

% Define fixed point
point = sys.fixedPoint([pi/2 pi/2 0 0]', 0);

% Derive time invariant H-Inf controller with uncertainty in velocities
controller = sys.tihinf(point, 'g', 50);

% Simulate fixed-point balance from perturbed initial conditions
response = sys.simulate([0 5], point.states + randn(4, 1)/25, ...
	'Controller', controller);

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

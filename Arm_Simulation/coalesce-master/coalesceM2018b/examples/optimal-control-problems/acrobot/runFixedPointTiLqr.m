%RUNFIXEDPOINTTILQR Stabilizes a fixed point on the acrobot using LQR.
%
% Description:
%   A time invariant linear quadratic regulator is designed around the
%		unstable fixed point for the acrobot. The controller is used to
%		stabilize the system from initial condition perturbations.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AcrobotSystem;

% Define fixed point
point = sys.fixedPoint([pi/2 pi/2 0 0]', 0);

% Derive time invariant LQR controller
controller = sys.tilqr(point);

% Simulate fixed-point balance from perturbed initial conditions
response = sys.simulate([0 5], point.states + randn(4, 1)/25, ...
	'Controller', controller);

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

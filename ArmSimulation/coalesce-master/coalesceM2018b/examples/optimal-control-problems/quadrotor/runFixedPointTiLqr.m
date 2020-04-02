%RUNFIXEDPOINTTILQR Runs quadrotor fixed point stabilization problem.
%
% Description:
%   A time invariant linear quadratic regulator is designed around a
%		fixed point for the quadrotor. The controller is used to
%		stabilize the system from initial condition perturbations.
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
model = QuadrotorSystem;

% Derive fixed point
point = model.fixedPoint([0 1 0 0 0 0]', [0 0]');

% Derive time invariant LQR controller
controller = model.tilqr(point);

% Simulate fixed-point balance from perturbed initial conditions
response = model.simulate([0 5], point.states + randn(6, 1)/2, ...
	'Controller', controller);

% Plot time response
response.plot;

% Construct and play animation
scene = QuadrotorScene(model, response);
Player(scene);

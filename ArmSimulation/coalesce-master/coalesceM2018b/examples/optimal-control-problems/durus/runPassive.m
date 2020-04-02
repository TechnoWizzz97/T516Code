%RUNPASSIVE Runs the passive Durus system.
%
% Copyright 2013-2014 Mikhail S. Jones

% Add function expression folder to path
addpath('dev');

% Construct system model
model = DurusSystem;

% Initial posture
x0 = [0; 0; -0.3; 0; -0.3; 0.6; -0.3; -0.9; 0.3; 0.3; 0; zeros(11,1)];

% Simulate passive system
response = model.simulate([0 5], x0, ...
	'initialMode', 'ds');

% Plot time response
response.plot;

% Construct and play animation
scene = DurusScene(model, response);
Player(scene);

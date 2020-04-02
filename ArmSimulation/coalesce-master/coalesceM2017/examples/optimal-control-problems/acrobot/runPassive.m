%RUNPASSIVE Runs passive acrobot.
%
% Description:
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = AcrobotSystem;

% Simulate passive system
response = sys.simulate([0 10], [pi/2 pi/2 1e-3 -1e-3]');

% Plot time response
response.plot;

% Construct and play animation
scene = AcrobotScene(sys, response);
Player(scene);

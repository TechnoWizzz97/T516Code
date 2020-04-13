%RUNPASSIVE Run passive Van der Pol oscillator.
%
% Description:
%
% Copyright 2013-2014 Mikhail S. Jones

% Construct system model
sys = VanDerPolSystem;

% Simulate passive system
response = sys.simulate([0 20], [2 0]);

% Plot time response
response.plot;

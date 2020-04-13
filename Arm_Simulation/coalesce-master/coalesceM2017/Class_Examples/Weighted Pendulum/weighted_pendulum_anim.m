clc;
clear all;
close all;

sys = Weighted_Pendulum_System;

[nlp, t, x, u] = sys.directCollocation(201);

nlp.addConstraint(-10, u, 10); % Torque Limit
nlp.addConstraint(0, x(1).initial, 0); % Initial Position
nlp.addConstraint(0, x(2).initial, 0); % Initial Velocity
nlp.addConstraint(pi/2, x(1).final, pi/2); % Final Position
nlp.addConstraint(0, x(2).final, 0); % Final Velocity
%nlp.addConstraint(tor, tor_abs, 0); % Absolute value constraint
%nlp.addConstraint(-tor, tor_abs, 0); % Absolute value constraint
nlp.addConstraint(0, t.final, 1); % Constrain time to 1 second

nlp.addObjective(trapz(u^2, t), 'Description','Minimize Integral torque^2');

optim = Ipopt(nlp);
optim.export;
optim.solve;

response = nlp.getResponse;

response.plot;

scene = Weighted_Pendulum_Scene(sys, response);
Player(scene);
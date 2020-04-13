%BRANINFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Branin function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several global
%   minima.
%
% Formula:
%   f(X) = a*(x2 - b*x1^2 + c*x1 - r)^2 + s*(1 - t)*cos(x1) + s
%
%   where,
%       a, b, c, r, s, and t are constants, and X is a vector of length d.
%
% Domain:
%   Typically evaluated on the square (-5 <= x1 <= 10), (0 <= x2 <= 15).
%
% Global Minimum:
%   f(X*) = 0.397887 at X* = [-pi,12.275], [pi,2.275], and [9.42478,2.475].
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Branin Function');

% Recommended variable values
a = 1;
b = 5.1/(4*pi^2);
c = 5/pi;
r = 6;
s = 10;
t = 1/(8*pi);

% Add design variables
x = nlp.addVariable([1 1], [-5 0], [10 15]);

% Add objective
nlp.addObjective(...
  a*(x(2) - b*x(1)^2 + c*x(1) - r)^2 + s*(1 - t)*cos(x(1)) + s);

% Display problem summary
nlp.display;

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Display solution
optim.display;

% Plot solution
nlp.plot

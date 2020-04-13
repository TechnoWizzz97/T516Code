%EGGHOLDERFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Eggholder function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = -(x2 + 47)*sin(sqrt(abs(x2 + x1/2 + 47))) - x1*sin(sqrt(abs(x1 -
%   (x2 + 47))))
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-512 <= X <= 512).
%
% Global Minimum:
%   f(512,404.2319) = -959.6407.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Eggholder Function');

% Add design variables
x = nlp.addVariable([1 1], [-512 -512], [512 512]);

% Add objective
nlp.addObjective(...
  -(x(2) + 47)*sin(sqrt(abs(x(2) + x(1)/2 + 47))) - ...
  x(1)*sin(sqrt(abs(x(1) - (x(2) + 47)))));

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

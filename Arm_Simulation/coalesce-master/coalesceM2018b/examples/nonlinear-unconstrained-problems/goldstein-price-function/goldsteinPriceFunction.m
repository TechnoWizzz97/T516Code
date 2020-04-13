%GOLDSTEINPRICEFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Goldstein-Price function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = (1 + (x1 + x2 + 1)^2*(19 - 14*x1 + 3*x1^2 - 14*x2 + 6*x1*x2 +
%   3*x2^2))*(30 + (2*x1 - 3*x2)^2*(18 - 32*x1 + 12*x1^2 + 48*x2 - 36*x1*x2
%   + 27*x2^2))
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-2 <= X <= 2).
%
% Global Minimum:
%   f(0,-1) = 3.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Goldstein-Price Function');

% Add design variables
x = nlp.addVariable([1 1], [-2 -2], [2 2]);

% Add objective
nlp.addObjective(...
  (1 + (x(1) + x(2) + 1)^2*(19 - 14*x(1) + 3*x(1)^2 - 14*x(2) + ...
  6*x(1)*x(2) + 3*x(2)^2))*(30 + (2*x(1) - 3*x(2))^2*(18 - 32*x(1) + ...
  12*x(1)^2 + 48*x(2) - 36*x(1)*x(2) + 27*x(2)^2)));

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

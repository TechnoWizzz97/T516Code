%CROSSINTRAYFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Cross-in-Tray function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several global
%   minima.
%
% Formula:
%   f(X) = -0.0001*(abs(sin(x1)*sin(x2)*exp(abs(100 - sqrt(x1^2 +
%   x2^2)/pi))) + 1)^0.1
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-10 <= X <= 10).
%
% Global Minimum:
%   f(X*) = -2.06261 at X* = [1.3491,-1.3491], [1.3491,1.3491],
%   [-1.3491,1.3491], and[-1.3491,-1.3491].
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Cross-in-Tray Function');

% Add design variables
x = nlp.addVariable([1 1], [-10 -10], [10 10]);

% Add objective
nlp.addObjective(...
  -0.0001*(abs(sin(x(1))*sin(x(2))*exp(abs(100 - sqrt(x(1)^2 + ...
  x(2)^2)/pi))) + 1)^0.1);

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

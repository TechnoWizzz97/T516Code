%BUKINN6FUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Bukin N6 function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = 100*sqrt(abs(x2 - 0.01*x1^2)) + 0.01*abs(x1 + 10)
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-15 <= x1 <= -5), (-3 <= x2 <= 3).
%
% Global Minimum:
%   f(-10,1) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Bukin N6 Function');

% Add design variables
x = nlp.addVariable([1 1], [-15 -3], [-5 3]);

% Add objective
nlp.addObjective(...
  100*sqrt(abs(x(2) - 0.01*x(1)^2)) + 0.01*abs(x(1) + 10));

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

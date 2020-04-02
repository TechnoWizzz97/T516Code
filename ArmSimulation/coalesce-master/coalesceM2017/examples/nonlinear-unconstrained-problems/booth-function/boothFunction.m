%BOOTHFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Booth function is commonly used as a performance test for
%   unconstrained optimization algorithms.
%
% Formula:
%   f(X) = (x1 + 2*x2 - 7)^2 + (2*x1 + x2 - 5)^2
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-10 <= X <= 10).
%
% Global Minimum:
%   f(1,3) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Booth Function');

% Add design variables
x = nlp.addVariable([1 1], [-10 -10], [10 10]);

% Add objective
nlp.addObjective(...
  (x(1) + 2*x(2) - 7)^2 + (2*x(1) + x(2) - 5)^2);

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

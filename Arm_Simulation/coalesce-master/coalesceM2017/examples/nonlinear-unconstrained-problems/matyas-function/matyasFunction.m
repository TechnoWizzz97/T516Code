%MATYASFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Matyas function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = 0.26*(x1^2 + x2^2) - 0.48*x1*x2
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-10 <= X <= 10).
%
% Global Minimum:
%   f(0,0) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Matyas Function');

% Add design variables
x = nlp.addVariable([1 1], [-10 -10], [10 10]);

% Add objective
nlp.addObjective(0.26*(x(1)^2 + x(2)^2) - 0.48*x(1)*x(2));

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

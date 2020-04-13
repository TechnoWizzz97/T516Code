%ROSENBROCKFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Rosenbrock function is commonly used as a performance test for
%   unconstrained optimization algorithms.
%
% Formula:
%   f(X) = sum(100*(X(2:d) - X(1:d-1)^2)^2 + (X(1:d-1) - 1)^2)
%
%   where,
%       X is a vector of length d.
%
% Domain:
%   Typically evaluated on the hypercube (-5 <= X <= 10).
%
% Global Minimum:
%   d = 2  >>>  f(1,1) = 0,
%   d = 3  >>>  f(1,1,1) = 0,
%   d > 3  >>>  f(-1,1,...,1) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Rosenbrock Function');

% Function dimensionality (d >= 2)
d = 2;

% Add design variables
x = nlp.addVariable(zeros(d, 1), repmat(-5, d, 1), repmat(10, d, 1));

% Add objective
nlp.addObjective(100*(x(2:d) - x(1:d-1)^2)^2 + (1 - x(1:d-1))^2);

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

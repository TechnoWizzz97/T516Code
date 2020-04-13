%SPHEREFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Sphere function is commonly used as a performance test for
%   unconstrained optimization algorithms.
%
% Formula:
%   f(X) = sum(X^2)
%
%   where,
%       X is a vector of length d.
%
% Domain:
%   Typically evaluated on the hypercube (-5.12 <= X <= 5.12).
%
% Global Minimum:
%   d >= 1  >>>  f(0,...,0) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Sphere Function');

% Function dimensionality (d >= 1)
d = 2;

% Add design variables
% x = nlp.addVariable(1, -5.12, 5.12, 'Length', d);
x = nlp.addVariable(repmat(1, d, 1), repmat(-5.12, d, 1), repmat(5.12, d, 1));

% Add objective
nlp.addObjective(x'*x);

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

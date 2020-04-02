%DROPWAVEFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Drop-Wave function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = -(1 + cos(12*sqrt(x1^2 + x2^2)))/(0.5*(x1^2 + x2^2) + 2)
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-5.12 <= X <= 5.12).
%
% Global Minimum:
%   f(0,0) = -1.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Drop-Wave Function');

% Add design variables
x = nlp.addVariable([1 1], [-5.12 -5.12], [5.12 5.12]);

% Add objective
nlp.addObjective(...
  -(1 + cos(12*sqrt(x(1)^2 + x(2)^2)))/(0.5*(x(1)^2 + x(2)^2) + 2));

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

%BEALEFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Beale function is commonly used as a performance test for
%   unconstrained optimization algorithms.
%
% Formula:
%   f(X) = (1.5 - x1 + x1*x2)^2 + (2.25 - x1 + x1*x2^2)^2 + (2.625 - x1 +
%   x1*x2^3)^2
%
%   where,
%       X is a vector of length 2.
%
% Domain:
%   Typically evaluated on the square (-4.5 <= X <= 4.5).
%
% Global Minimum:
%   f(3,0.5) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Beale Function');

% Add design variables
x = nlp.addVariable([1 1], [-4.5 -4.5], [4.5 4.5]);

% Add objective
nlp.addObjective(...
  (1.5 - x(1) + x(1)*x(2))^2 + ...
  (2.25 - x(1) + x(1)*x(2)^2)^2 + ...
  (2.625 - x(1) + x(1)*x(2)^3)^2);

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

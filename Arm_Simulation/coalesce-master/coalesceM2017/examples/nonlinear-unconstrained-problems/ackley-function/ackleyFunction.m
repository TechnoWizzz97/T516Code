%ACKLEYFUNCTION Nonlinear unconstrained optimization test function.
%
% Description:
%   The Ackley function is commonly used as a performance test for
%   unconstrained optimization algorithms. The function has several local
%   minima.
%
% Formula:
%   f(X) = -a*exp(-b*sqrt(1/d*sum(X^2))) - exp(1/d*sum(cos(c*X))) + a +
%   exp(1)
%
%   where,
%       a, b, and c are constants, and X is a vector of length d.
%
% Domain:
%   Typically evaluated on the hypercube (-32.768 <= X <= 32.768).
%
% Global Minimum:
%   d >= 1  >>>  f(0,...,0) = 0.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Construct optimization problem object
nlp = Nlp('Name', 'Ackley Function');

% Recommended variable values
a = 20;
b = 0.2;
c = 2*pi;

% Function dimensionality (d >= 1)
d = 2;

% Add design variables
x = nlp.addVariable(ones(d, 1), repmat(-32.768, d, 1), repmat(32.768, d, 1));

% Add objective
nlp.addObjective(...
	-a*exp(-b*sqrt(1/d*x'*x)) - exp(1/d*cos(c*x'*x)) + a + exp(1));

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

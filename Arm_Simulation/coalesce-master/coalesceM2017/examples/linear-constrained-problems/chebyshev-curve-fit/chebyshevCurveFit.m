%CHEBYSHEVCURVEFIT Chebyshev polynomial curve fit.
%
% Description:
%		This script implements a linear program to find the best Chebyshev
%		polynomial to fit a set of data. The problem is defined in epigraph
%		form, as the objective consists of a single variable the constraints
%		define the true objective. The resulting problem gradients are dense.
%
% Copyright 2014 Mikhail S. Jones and Jonathan Van Why

% Clean up the workspace
clear all; clc;

% Define number of samples
N = 100;

% Define degree of the fit polynomial
d = 10;

% Define sample data for Chebyshev polynomial fit
x = linspace(-2*pi, 2*pi, N);
data = cos(x) + randn(1, numel(x))/100;

% Construct problem object
nlp = Nlp('Name', 'Chebyshev Polynomial Curve Fit');

% Construct numeric matrix of Chebyshev basis values
TVals(d+1,N) = 0; % Pre-allocate
TVals(1,:) = 1;
TVals(2,:) = x;
for i = 2:d
	TVals(i+1,:) = 2*x.*TVals(i,:) - TVals(i-1,:);
end % for

% Construct cell array of parameters describing each row of the Chebyshev
% basis values
% T = nlp.addParameter([1, d+1, N], TVals);
T = nlp.addVariable([1, d+1], TVals, TVals);

% Add slack variable for the minimax operation
m = nlp.addVariable(1, 1, -inf, inf);

% Add weights for the Chebyshev basis polynomials
p = nlp.addVariable(d+1, 1, -inf, inf);

% Sample the polynomials, the values at the sampling points are linear
% combinations (defined by p) of the Chebyshev basis values. Evaluate the
% differences between the polynomial and the data to be approximated at
% the sample points
err = data - T*p;

% Add minimax objective
nlp.addObjective(m);

% Apply maximality constraints
nlp.addConstraint(0, m - err, inf);
nlp.addConstraint(0, m + err, inf);

% Export and solve
optim = Ipopt(nlp);
optim.export;
optim.solve;

% Plot sparsity structure to show denseness
nlp.plotSparsity;

% Grab the solution, and do some data processing
polyVals = [eval(p)]*TVals;

% Plot curve fit over original data
figure; hold on; grid on; box on;
plot(x, data, '.k');
plot(x, polyVals, 'r');
legend('Data', 'Chebyshev Fit');
xlabel('Samples');
ylabel('Values');

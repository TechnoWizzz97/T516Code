%FENCE Runs maximum area fence problem.
%
% Description:
%   Find the fence shape that maximizes area given a fixed perimeter.
%
% Copyright 2013-2014 Mikhail S. Jones

% Clean up the workspace
clear all; clc;

% Number of fence segments
N = 100;

% Construct optimization problem
nlp = Nlp('Name', 'Fence Problem');

% Add variables
x = nlp.addVariable(1, -Inf, Inf, 'Length', N+1);
y = nlp.addVariable(0, -Inf, Inf, 'Length', N+1);
A = nlp.addVariable(0, -Inf, Inf, 'Length', N+1);

% Add integral constraint by formulating as PDE cumulative area constraint
nlp.addPdeConstraint(A, y, x, 'Method', 'Implicit Euler');

% Add perimeter constraint
dx = ind(x,2:N+1) - ind(x,1:N);
dy = ind(y,2:N+1) - ind(y,1:N);
nlp.addConstraint(0, dx^2 + dy^2, 1^2);

% Add initial value constraints
nlp.addConstraint(0, ind(x,1), 0);
nlp.addConstraint(0, ind(y,1), 0);
nlp.addConstraint(0, ind(x,N+1), 0);
nlp.addConstraint(0, ind(y,N+1), 0);
nlp.addConstraint(0, ind(A,1), 0);

% Add maximum area objective
nlp.addObjective(-ind(A,N+1));

% Construct optimizer interface object, export and solve
optim = Snopt(nlp);
optim.export;
optim.solve;

% Plot optimal fence shape
figure; hold on; grid on; box on; axis equal;
plot(squeeze(eval(x)), squeeze(eval(y)), 'r.-');

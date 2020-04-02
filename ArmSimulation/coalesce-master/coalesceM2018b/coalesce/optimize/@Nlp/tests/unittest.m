% Clear workspace
clear all; clc;

% Construct a test problem
nlp = Nlp('Name', 'Test Problem', 'Description', 'Test Problem');

% Add parameters
ps = nlp.addParameter(1, 'Description', 'Scalar Parameter');
pv = nlp.addParameter(ones(2,1), 'Description', 'Scalar Parameter Array');

% Add variables
vs = nlp.addVariable(1, 0, Inf, 'Description', 'Scalar Variable');
vv = nlp.addVariable(eye(2), zeros(2), Inf(2), 'Description', 'Scalar Variable Array');

% Add objective
nlp.addObjective(vs, 'Description', 'Test Objective');

% Add constraint
nlp.addConstraint(0, ps*vs, 0);
nlp.addConstraint(0, ps*vv, 0);
nlp.addConstraint(0, pv*vs, 0);
nlp.addConstraint(0, pv'*vv*pv, 0);

% Display problem properties
nlp.display

optim = Ipopt(nlp);

function solve(this)
%SOLVE Solve optimization problem using MATLAB FMINCON.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Initializing FMINCON solver... \n');

	% Refresh function and file system caches
	rehash;

	% Evaluate fminconUser function to determine constants
	[A, b, Aeq, beq] = fminconUser;
	lb = this.nlp.variableLowerBound;
	ub = this.nlp.variableUpperBound;
	x0 = this.nlp.initialGuess;

	% Run FMINCON
	[x, f, this.info] = fmincon('fminconObj', ...
		x0, A, b, Aeq, beq, lb, ub, 'fminconNonlcon', this.options);

	% Parse solution back into variable objects
	this.nlp.solution = x;

	% Send desktop notification
	message = ['FMINCON optimizer finished.'...
		'\n\t* Exit flag: ' num2str(this.info)...
		'\n\t* Objective value: ' num2str(f)];
	notify(message, 'COALESCE', 1);
end % solve

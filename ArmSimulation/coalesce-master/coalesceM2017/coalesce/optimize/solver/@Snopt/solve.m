function solve(this)
%SOLVE Solve optimization problem using SNOPT.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Initializing SNOPT solver... \n');

	% Objective function information
	ObjAdd = 0; ObjRow = 1;

	% Refresh function and file system caches
	rehash;

	% Evaluate snoptUser function to determine constants
	[A, iAfun, jAvar, iGfun, jGvar] = snoptUser;
	xlow = this.nlp.variableLowerBound';
	xupp = this.nlp.variableUpperBound';
	Flow = [this.nlp.objective.lowerBound this.nlp.constraint.lowerBound]';
	Fupp = [this.nlp.objective.upperBound this.nlp.constraint.upperBound]';
	x0 = this.nlp.initialGuess';

	% Run SNOPT
	[x, f, this.info] = snopt(x0, xlow, xupp, Flow, Fupp, ...
		'snoptUserFun', ObjAdd, ObjRow, A, iAfun, jAvar, iGfun, jGvar);

	% Parse solution back into variable objects
	this.nlp.solution = x';

	% Send desktop notification
	message = ['SNOPT optimizer finished.'...
		'\n\t* Exit flag: ' num2str(this.info)...
		'\n\t* Objective value: ' num2str(f(ObjRow) + ObjAdd)];
	notify(message, 'COALESCE', 1);
end % solve

function solve(this)
%SOLVE Solve optimization problem using IPOPT.
%
% Copyright 2013-2014 Mikhail S. Jones

	% User feedback
	fprintf('Initializing IPOPT solver... \n');

	% Refresh function and file system caches
	rehash;

	% Set IPOPT inputs
	this.options.lb = this.nlp.variableLowerBound;
	this.options.ub = this.nlp.variableUpperBound;
	this.options.cl = [this.nlp.constraint.lowerBound];
	this.options.cu = [this.nlp.constraint.upperBound];
	% funcs.iterfunc = @ipoptIterFunc;
	funcs.objective = @ipoptObjective;
	funcs.gradient = @ipoptGradient;
	funcs.constraints = @ipoptConstraints;
	funcs.jacobian = @ipoptJacobian;
	funcs.jacobianstructure = @ipoptJacobianStructure;
	% funcs.hessian = @ipoptHessian;
	% funcs.hessianstructure = @ipoptHessianStructure;

	% Run IPOPT
	[x, this.info] = ipopt(this.nlp.initialGuess, funcs, this.options);

	% Parse solution back into variable objects
	this.nlp.solution = x;

	% Send desktop notification
	message = ['IPOPT optimizer finished.'...
		'\n\t* Exit flag: ' num2str(this.info.status)...
		'\n\t* Objective value: ' num2str(ipoptObjective(x))];
	notify(message, 'COALESCE', 1);
end % solve

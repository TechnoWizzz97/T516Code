function expression = addVariable(this, initialGuess, lowerBound, upperBound, varargin)
%ADDVARIABLE Add a design variable to the optimization problem.
%
% Syntax:
%   obj.addVariable(initialGuess, lowerBound, upperBound)
%
% Required Input Arguments:
%   initialGuess - (DOUBLE) Initial guess for optimization solver
%   lowerBound - (DOUBLE) Variable lower bound
%   upperBound - (DOUBLE) Variable upper bound
%
% Optional Input Arguments:
%   description - (CHAR) Variable description
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct input argument parser
	parser = inputParser;
	parser.addRequired('initialGuess', ...
		@(x) validateattributes(x, ...
			{'double'}, {}));
	parser.addRequired('lowerBound', ...
		@(x) validateattributes(x, ...
			{'double'}, {}));
	parser.addRequired('upperBound', ...
		@(x) validateattributes(x, ...
			{'double'}, {}));
	parser.addParamValue('description', 'No Name', ...
		@(x) validateattributes(x, ...
			{'char'}, {'vector'}));
	parser.addParamValue('length', 1, ...
		@(x) validateattributes(x, ...
			{'double'}, {'scalar', 'positive', 'integer'}));

	% Parse input arguments
	parser.parse(initialGuess, lowerBound, upperBound, varargin{:});

	% Store the results in a convenient structure
	opts = parser.Results;

	% Compute dimensions
	igSize = size(initialGuess);
	lbSize = size(lowerBound);
	ubSize = size(upperBound);

	% Check dimensions match
	if all(igSize(1:2) ~= lbSize(1:2)) || all(igSize(1:2) ~= ubSize(1:2))
		error('Dimensions do not match.');
	end % if

	% User feedback
	fprintf('Adding ([\b%d]\bx[\b%d]\bx[\b%d]\b) [\b%s]\b variable... \n', ...
		igSize(1:2), opts.length, opts.description);

	% Construct variables
	for d2 = 1:igSize(2)
		for d1 = 1:igSize(1)
			% Compute number of variables
			nVars = this.numberOfVariables;

			% Construct variable
			expression(d1,d2) = Variable('var', nVars + (1:opts.length));
			expression(d1,d2).nlp = this;
			expression(d1,d2).initialGuess = initialGuess(d1,d2,:);
			expression(d1,d2).solution = expression(d1,d2).initialGuess;
			expression(d1,d2).lowerBound = lowerBound(d1,d2,:);
			expression(d1,d2).upperBound = upperBound(d1,d2,:);
			expression(d1,d2).description = opts.description;

			% Append variable object array
			this.variable(end+1) = expression(d1,d2);
		end % for
	end % for
end % addVariable

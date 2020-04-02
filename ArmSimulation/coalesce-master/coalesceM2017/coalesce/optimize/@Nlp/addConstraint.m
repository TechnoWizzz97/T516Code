function addConstraint(this, lowerBound, expression, upperBound, varargin)
%ADDCONSTRAINT Add constraint to problem structure.
%
% Syntax:
%   obj.addConstraint(lowerBound, expression, upperBound)
%
% Required Input Arguments:
%   lowerBound - (DOUBLE) Design variable lower bounds
%   expression - (EXPRESSIONNODE) Symbolic constraint expression
%   upperBound - (DOUBLE) Design variable upper bounds
%
% Optional Input Arguments:
%   description - (CHAR) Description for identification
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct input argument parser
	parser = inputParser;
	parser.addParamValue('description', 'No Name', ...
		@(x) validateattributes(x, ...
			{'char'}, {'vector'}));

	% Parse input arguments
	parser.parse(varargin{:});

	% Store the results in a convenient structure
	opts = parser.Results;

	% User feedback
	fprintf('Adding ([\b%d]\b) [\b%s]\b constraints... \n', ...
		sum(sum([expression.length])), opts.description);

	for i = 1:numel(expression)
		% Check if all variables in objective have been defined
		if ~this.checkVariables(expression(i))
		  error('Not all variables in constraint expression have been defined.');
		end % if

		% Append object array with more constraints
		this.constraint(end+1) = Constraint(expression(i), ...
			lowerBound, upperBound, opts.description);
	end % for
end % addConstraint

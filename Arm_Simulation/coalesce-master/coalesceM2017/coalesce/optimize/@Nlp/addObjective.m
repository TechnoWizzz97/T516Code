function addObjective(this, expression, varargin)
%ADDOBJECTIVE Adds objective to problem structure.
%
% Syntax:
%   obj.addObjective(expression)
%
% Required Input Arguments:
%   expression - (EXPRESSIONNODE) Symbolic objective expression
%
% Optional Input Arguments:
%   description - (CHAR) Objective description
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
	fprintf('Adding ([\b1]\b) [\b%s]\b objective... \n', opts.description);

	% Verify data type
	expression = ConstantNode(0) + expression;

	% Check if all variables in objective have been defined
	if ~this.checkVariables(expression)
	  error('Not all variables in objective expression have been defined.');
	end % if

	% Check number of objective objects
	if numel(this.objective) >= 1
		error('Multi-objective problem formulations are not supported.');
	end % if

	% Store objective function in problem structure
	this.objective(end+1) = Objective(expression, opts.description);
end % addObjective

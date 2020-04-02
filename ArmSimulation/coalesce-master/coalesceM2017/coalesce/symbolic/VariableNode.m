%VARIABLENODE Expression tree variable node subclass.
%
% TODO: Use length input and have a core engine determine available indexes
% TODO: diff or trapz or some equivalent so user doesn't have use ind
%
% Copyright 2014 Mikhail S. Jones

classdef VariableNode < ExpressionNode

	% PROTECTED PROPERTIES ==================================================
	properties (SetAccess = protected)
		name@char vector
		index@double vector
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function this = VariableNode(name, index)
		%VARIABLENODE Variable node constructor.

			% Check number of input arguments
			switch nargin
			case 1
				% Construct scalar variable
				this.index = 1;

			case 2
				% Check length is positive integer
				if all(index > 0) && all(mod(index,1) == 0)
					% Construct vector variable with specified indexes
					this.index = index;
				else
					error('Indexes must be a positive integers.');
				end % if

			otherwise
				error('Invalid number of input arguments.');
			end % switch

			% Set object properties
			this.name = name;
			this.length = length(this.index);
			this.isSimple = true;
		end % VariableNode

		function this = ind(this, index)
		%IND Index expression.

			% Copy object and set new index values
			for i = 1:numel(this)
				this(i).index = this(i).index(index);
				this(i).length = length(this(i).index);
			end % for
		end % ind

		function this = initial(this)
		%INITIAL Initial index expression.

			% Copy object and set new index values
			for i = 1:numel(this)
				this(i).index = this(i).index(1);
				this(i).length = 1;
			end % for
		end % initial

		function this = final(this)
		%FINAL Final index expression.

			% Copy object and set new index values
			for i = 1:numel(this)
				this(i).index = this(i).index(end);
				this(i).length = 1;
			end % for
		end % initial
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function y = diff_(this, x)
		%DIFF_ Overloaded abstract method to differentiate variable node.

			% Check if differentiation variable matches variable node
			if this == x
				% The derivative of a variable with respect to itself is one
				y = ConstantNode(1);
			else
				% The derivative of a variable with respect to another is zero
				y = ConstantNode(0);
			end % if
		end % diff_

		function y = eval_(this)
		%EVAL_ Overloaded abstract method to evaluate variable node.

			% A variable node can not be evaluated
			error('Cannot evaluate variable nodes.');
		end % eval_

		function this = simplify_(this)
		%SIMPLIFY_ Overloaded abstract method to simplify variable node.

			% A variable node cannot not be simplified anymore
		end % simplify_

		function vars = symvar_(this)
		%SYMVAR_ Determine the variables in an expression tree.

			vars = this;
		end % symvar_

		function str = char_(this)
		%CHAR_ Overloaded abstract method to convert variable node to char.

			% Convert into indexed variable
			str = [this.name '(' indexHelper(this.index) ')'];
		end % char_

		function str = matlabCode_(this)
		%MATLABCODE_ Overloaded abstract method to convert node to matlab code.

			% Convert into indexed variable
			str = [this.name '(' indexHelper(this.index) ')'];
		end % matlabCode_

		function str = fortranCode_(this)
		%FORTRANCODE_ Overloaded abstract method to convert node to fortran code.

			% Convert into indexed variable
			str = [this.name '(' indexHelper(this.index) ')'];
		end % fortranCode_
	end % methods
end % classdef

function str = indexHelper(index)
%INDEXHELPER Convert vector to shorthand index.

	% Check if index is all the same
	if all(diff(index) == 0)
		% Keep index in shorthand scalar form (2)
		str = sprintf('%d', index(1));

	% Check if index is monotonically increasing
	elseif all(diff(diff(index)) == 0)
		% Keep index in shorthand vectorized form (2:5)
		str = sprintf('%d:%d:%d', index(1), index(2) - index(1), index(end));

	else
		% Expand index into explicit vector ([2,3,4,5])
		str = sprintf('%d ', reshape(index, 1, []));
		str = str(1:end-1);
	end % if
end % indexHelper

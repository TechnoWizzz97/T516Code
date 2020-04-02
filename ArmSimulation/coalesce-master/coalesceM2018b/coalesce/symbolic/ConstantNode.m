%CONSTANTNODE Expression tree constant node subclass.
%
% Copyright 2014 Mikhail S. Jones

classdef ConstantNode < ExpressionNode

	% PROTECTED PROPERTIES ==================================================
	properties (SetAccess = protected)
		value@double scalar
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function this = ConstantNode(value)
		%CONSTANTNODE Constant node constructor.

			% Set object properties
			this.value = value;
			this.length = 1;
			this.isSimple = true;
		end % ConstantNode

		function this = ind(this, index)
		%IND Index expression.
		end % ind
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function y = diff_(this, x)
		%DIFF_ Overloaded abstract method to differentiate constant node.

			% The derivative of a constant is always zero
			y = ConstantNode(0);
		end % diff_

		function y = eval_(this)
		%EVAL_ Overloaded abstract method to evaluate constant node.

			% Return constant value
			y = this.value;
		end % eval_

		function this = simplify_(this)
		%SIMPLIFY_ Overloaded abstract method to simplify constant node.

			% A constant node cannot not be simplified anymore
		end % simplify_

		function vars = symvar_(this)
		%SYMVAR_ Determine the variables in an expression tree.

			vars = [];
		end % symvar_

		function str = char_(this)
		%CHAR_ Overloaded abstract method to convert constant node to char.

			% Convert numeric value to string keeping double precision
			str = sprintf('%g', this.value);
		end % char_

		function str = matlabCode_(this)
		%MATLABCODE_ Overloaded abstract method to convert node to matlab code.

			% Convert numeric value to string keeping double precision
			str = sprintf('%.*f', ceil(-log10(eps(this.value))), this.value);

			% Remove trailing zeros
			tmp = regexp(str, '^0+(?!\.)|(?<!\.)0+$', 'split');
			str = tmp{1};
		end % matlabCode_

		function str = fortranCode_(this)
		%FORTRANCODE_ Overloaded abstract method to convert node to fortran code.

			% Convert to string with double precision
			str = sprintf('%.*f', ceil(-log10(eps(this.value))), this.value);

			% Remove trailing zeros
			tmp = regexp(str, '^0+(?!\.)|(?<!\.)0+$', 'split');
			str = [tmp{1}, 'd+0'];
		end % fortranCode_
	end % methods
end % classdef

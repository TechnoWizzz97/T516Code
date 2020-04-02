%MINUSBINARYOPERATORNODE Binary minus operator node subclass.
%
% Copyright 2014 Mikhail S. Jones

classdef MinusBinaryOperatorNode < BinaryOperatorNode

	% PUBLIC PROPERTIES =====================================================
	methods
		function this = MinusBinaryOperatorNode(left, right)
		%MINUSBINARYOPERATORNODE Binary minus operator node constructor.
			this = this@BinaryOperatorNode(left, right);
		end % MinusBinaryOperatorNode
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function y = diff_(this, x)
		%DIFF_ Overloaded abstract method to differentiate node.

			% Apply chain rule
			y = MinusBinaryOperatorNode(diff_(this.left, x), diff_(this.right, x));
		end % diff_

		function y = eval_(this)
		%EVAL_ Overloaded abstract method to evaluate node.
			y = eval_(this.left) - eval_(this.right);
		end % eval_

		function this = simplify_(this)
		%SIMPLIFY_ Overloaded abstract method with to simplify node.

			if this.isSimple
				return;
			else
				this.left = simplify_(this.left);
				this.right = simplify_(this.right);
				this.isSimple = true;
			end % if

			if isa(this.left, 'ConstantNode') && isa(this.right, 'ConstantNode')
				this = ConstantNode(eval_(this));

			elseif isa(this.left, 'ConstantNode')
				if this.left.value == 0
					% Apply simplification rule (0 - x = -x)
					this = MinusUnaryOperatorNode(this.right);
				end % if

			elseif isa(this.right, 'ConstantNode')
				if this.right.value == 0
					% Apply simplification rule (x - 0 = x)
					this = this.left;
				end % if

			elseif isequal(this.left, this.right)
				% Apply simplification rule (x - x = 0)
				this = ConstantNode(0);
			end % if
		end % simplify_

		function str = char_(this)
		%CHAR_ Overloaded abstract method to convert node to char.
			str = ['(' char_(this.left) '-' char_(this.right) ')'];
		end % char_

		function str = matlabCode_(this)
		%MATLABCODE_ Overloaded abstract method to convert node to matlab code.
			str = ['(' matlabCode_(this.left) '-' matlabCode_(this.right) ')'];
		end % matlabCode_

		function str = fortranCode_(this)
		%FORTRANCODE_ Overloaded abstract method to convert node to fortran code.
			str = ['(' fortranCode_(this.left) '-' fortranCode_(this.right) ')'];
		end % fortranCode_
	end % methods
end % classdef

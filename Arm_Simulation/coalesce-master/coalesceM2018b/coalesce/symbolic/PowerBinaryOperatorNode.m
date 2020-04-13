%DIVIDEBINARYOPERATORNODE Binary division operator node subclass.
%
% Copyright 2014 Mikhail S. Jones

classdef PowerBinaryOperatorNode < BinaryOperatorNode

	% PUBLIC PROPERTIES =====================================================
	methods
		function this = PowerBinaryOperatorNode(left, right)
		%POWERBINARYOPERATORNODE Binary power operator node constructor.
			this = this@BinaryOperatorNode(left, right);
		end % PowerBinaryOperatorNode
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function y = diff_(this, x)
		%DIFF_ Overloaded abstract method to differentiate node.

			% Apply chain rule
			leftDiff = diff_(this.left, x);
			rightDiff = diff_(this.right, x);

			% Check if either is zero
			if isa(leftDiff, 'ConstantNode')
				leftZero = eval_(leftDiff) == 0;
			else
				leftZero = false;
			end % if
			if isa(rightDiff, 'ConstantNode')
				rightZero = eval_(rightDiff) == 0;
			else
				rightZero = false;
			end % if

			if leftZero && rightZero
				y = ConstantNode(0);
			elseif rightZero
				y = this.right.*leftDiff.*(this.left.^(this.right - 1));
			else
				y = this.left.^this.right.*(leftDiff.*this.right./this.left + rightDiff.*log(this.left));
			end % if
		end % diff_

		function y = eval_(this)
		%EVAL_ Overloaded abstract method to evaluate node.
			y = eval_(this.left).^eval_(this.right);
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
				this = ConstantNode(eval(this));

			elseif isa(this.left, 'ConstantNode')
				if this.left.value == 1
					% Apply simplification rule (1^x = 1)
					this = ConstantNode(1);
				end % if

			elseif isa(this.right, 'ConstantNode')
				if this.right.value == 0
					% Apply simplification rule (x^0 = 1)
					this = ConstantNode(1);

				elseif this.right.value == 1
					% Apply simplification rule (x^1 = x)
					this = this.left;

				elseif this.right.value == -1
					% Apply simplification rule (x^-1 = 1 / x)
					this = DivideBinaryOperatorNode(ConstantNode(1), this.left);
				end % if
			end % if

			% TODO: Apply simplification rule (x^2^3 = x^6)
		end % simplify_

		function str = char_(this)
		%CHAR_ Overloaded abstract method to convert node to char.
			str = [char_(this.left) '.^' char_(this.right)];
		end % char_

		function str = matlabCode_(this)
		%MATLABCODE_ Overloaded abstract method to convert node to matlab code.
			str = ['(' matlabCode_(this.left) '.^' matlabCode_(this.right) ')'];
		end % matlabCode_

		function str = fortranCode_(this)
		%FORTRANCODE_ Overloaded abstract method to convert node to fortran code.
			str = ['(' fortranCode_(this.left) '**' fortranCode_(this.right) ')'];
		end % fortranCode_
	end % methods
end % classdef

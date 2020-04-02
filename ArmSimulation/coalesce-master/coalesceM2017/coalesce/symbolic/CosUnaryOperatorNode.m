%COSUNARYOPERATORNODE Unary cosine operator node subclass.
%
% Copyright 2014 Mikhail S. Jones

classdef CosUnaryOperatorNode < UnaryOperatorNode

	% PUBLIC PROPERTIES =====================================================
	methods
		function this = CosUnaryOperatorNode(operand)
		%COSUNARYOPERATORNODE Unary cosine operator node constructor.
			this = this@UnaryOperatorNode(operand);
		end % CosUnaryOperatorNode
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function d = diff_(this, var)
		%DIFF_ Overloaded abstract method to differentiate node.

			% Compute derivative of operand
			operandDiff = diff_(this.operand, var);

			% Apply chain rule
			d = -sin(this.operand)*operandDiff;
		end % diff_

		function val = eval_(this)
		%EVAL_ Overloaded abstract method to evaluate node.
			val = cos(eval_(this.operand));
		end % eval_

		function this = simplify_(this)
		%SIMPLIFY_ Overloaded abstract method to simplify node.

			% Simplify the operand
			this.operand = simplify_(this.operand);

			if isa(this.operand, 'ConstantNode')
				% Evaluate operator numerically
				this = ConstantNode(eval_(this), this.length);
			end % if
		end % simplify_

		function str = char_(this)
		%CHAR_ Overloaded abstract method to convert node to char.
			str = ['cos(' char_(this.operand) ')'];
		end % char_

		function str = matlabCode_(this)
		%MATLABCODE_ Overloaded abstract method to convert node to matlab code.
			str = ['cos(' matlabCode_(this.operand) ')'];
		end % matlabCode_

		function str = fortranCode_(this)
		%FORTRANCODE_ Overloaded abstract method to convert node to fortran code.
			str = ['cos(' fortranCode_(this.operand) ')'];
		end % fortranCode_
	end % methods
end % classdef

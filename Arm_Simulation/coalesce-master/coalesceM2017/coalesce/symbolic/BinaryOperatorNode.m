%BINARYOPERATORNODE Expression tree binary operator node subclass.
%
% Copyright 2014 Mikhail S. Jones

classdef BinaryOperatorNode < ExpressionNode

	% PROTECTED PROPERTIES ==================================================
	properties (Access = protected)
		left@ExpressionNode
		right@ExpressionNode
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function this = BinaryOperatorNode(left, right)
		%BINARYOPERATORNODE Binary operator node constructor.

			% Check data type of left node
			if isa(left, 'ExpressionNode')
				% Expression node subclasses can be stored as is
				this.left = simplify_(left);
			else
				% Convert into appropriate ExpressionNode subclass
				this.left = this.convertObject('', left);
			end % if

			% Check data type of right node
			if isa(right, 'ExpressionNode')
				% Expression node subclasses can be stored as is
				this.right = simplify_(right);
			else
				% Convert into appropriate ExpressionNode subclass
				this.right = this.convertObject('', right);
			end % if

			% Check internal dimensions
			if this.left.length ~= this.right.length
				if this.left.length ~= 1 && this.right.length ~= 1
					error('Dimensions do not match.');
				end % if
			end % if

			% Set object properties
			this.length = max(this.left.length, this.right.length);
		end % BinaryOperatorNode

		function this = ind(this, index)
		%IND Index expression.

			% Copy object and set new index values
			for i = 1:numel(this)
				this(i).left = ind(this(i).left, index);
				this(i).right = ind(this(i).right, index);
				this(i).length = length(index);
			end % for
		end % ind
	end % methods

	% PROTECTED METHODS =====================================================
	methods (Access = protected)
		function vars = symvar_(this)
		%SYMVAR_ Determine the variables in an expression tree.

			% Determine variables in branches
			leftVars = symvar_(this.left);
			rightVars = symvar_(this.right);

			% Create uniqueness index
			ind = ones(size(rightVars));

			% Return only unique variables
			for i = 1:numel(rightVars)
				for j = 1:numel(leftVars)
					if rightVars(i) == leftVars(j)
						ind(i) = 0;
					end % if
				end % for
			end % for

			vars = [leftVars rightVars(find(ind))];
		end % symvar_
	end % methods
end % classdef

%EXPRESSIONNODE Expression tree node abstract superclass.
%
% Copyright 2014 Mikhail S. Jones

classdef (Abstract = true) ExpressionNode < matlab.mixin.Heterogeneous

	% PROTECTED PROPERTIES ==================================================
	properties (SetAccess = protected)
		length@double scalar = 0
		isSimple@logical scalar = false
	end % properties

	% ABSTRACT METHODS ======================================================
	methods (Abstract = true, Access = protected)
		% Expression graph
		% reverseAutoDiff_ % Reverse automatic differentiation
		% forwardAutoDiff_ % Rorward automatic differentiation
		% sourceAutoDiff_ % Source transformation automatic differentiation
		% symbolicDiff_ % Symbolic differentiation
		diff_
		eval_
		simplify_
		symvar_

		% Conversion
		char_
		matlabCode_
		fortranCode_
		% cCode_
	end % methods

  % SEALED METHODS ========================================================
	methods (Sealed = true)
		function display(this)
		%DISPLAY Display expression node properties.

			% Determine object array size
			[m, n] = size(this);

			% Display
			fprintf('\n(%dx%d) Expression tree.\n\n', m, n);
		end % display

		function [i, j, s] = jacobian(this)
		%JACOBIAN Computes the jacobian matrix.

			% Initialize cell arrays
			i = {}; j = {}; s = ConstantNode.empty;

			% Counter
			c = 1;

			% Loop through expressions
			for iter = 1:numel(this)
				% Find all unique variable nodes in tree
				vars = symvar(this(iter));

				% Loop though variables
				for k = 1:numel(vars)
					% Compute partial derivative
					l = max(this(iter).length, vars(k).length);
					i{end+1}(1,1:l) = c:(c - 1 + this(iter).length);
					j{end+1}(1,1:l) = vars(k).index;
					s(end+1) = simplify(diff(this(iter), vars(k)));
					s(end).length = l;
				end % for

				% Advance counter
				c = c + this(iter).length;
			end % for
		end % jacobian
	end % methods

	% STATIC METHODS ========================================================
	methods (Access = protected, Sealed = true, Static = true)
		function this = getDefaultScalarElement
		%GETDEFAULTSCALARELEMENT Default for heterogeneous arrays.

			% Constant scalar zeros
			this = ConstantNode(0);
		end % getDefaultScalarElement

		function this = convertObject(domClassName, objToConvert)
		%CONVERTOBJECT Heterogeneous array conversion rules.

			% Check class of object to convert
			switch class(objToConvert)
				case 'double'
					% Convert double to constant node
					for row = 1:size(objToConvert,1)
						for col = 1:size(objToConvert,2)
							this(row,col) = ConstantNode(objToConvert(row,col));
						end % for
					end % for

				otherwise
					error(['Unkown conversion rule for ' class(objToConvert) ' class.']);
			end % switch
		end % convertObject
	end % methods

	% SEALED METHODS ========================================================
	methods (Sealed = true)
		% SYMBOLIC OPERATIONS -------------------------------------------------
		function val = eval(this)
		%EVAL Evaluate expression tree.
			val = unaryHelper(@eval_, this);
		end % eval

		function D = diff(this, var)
		%DIFF Differentiate expression tree.
			D = unaryHelper(@(f) diff_(f, var), this);
		end % diff

		function S = simplify(this)
		%SIMPLIFY Simplify expression tree.
			S = unaryHelper(@simplify_, this);

			% TODO: Should change size but simplify_ can sometimes lose length while simplifying terms...
			for k = 1:numel(S)
				S(k).length = this(k).length;
			end % for
		end % simplify

		function vars = symvar(this)
			if numel(this) == 1
				vars = symvar_(this);
			else
				error('Array operation not yet implemented.');
			end % if
		end % symvar

		% CONVERSION OPERATIONS -----------------------------------------------
		function str = char(this)
		%CHAR Convert expression tree to char.

			if numel(this) == 1
				str = char_(this);
			else
				error('Array operation not yet implemented.');
			end % if
		end % char

		function matlabCode(this, name, fid)
		%MATLABCODE Convert expression tree to matlab code.

			% Check input arguments
			if nargin == 2; fid = 1; end % if

			% Determine size of expression vector
			m = sum([this.length]);

			if m == 0
				% Write empty double
				fprintf(fid, '\t%s = [];\n', name);
			else
				% Initialize index counter
				c = 1;

				% Write pre-allocation assignment
				fprintf(fid, '\t%s(%d,1) = 0; %% Pre-allocation\n', name, m);

				for i = 1:numel(this)
					% Write MATLAB code
					str = matlabCode_(this(i));

					if strcmp(str, '0.0')
						% Do nothing, should already be preallocated to zero
					else
						% Otherwise we can set its value
						if this(i).length == 1
							% Scalar index
							fprintf(fid, '\t%s(%d) = %s;\n', name, c, str);
						else
							% Vector index
							fprintf(fid, '\t%s(%d:%d) = %s;\n', ...
								name, c, c + this(i).length - 1, str);
						end % if
					end % if

					% Advance index counter
					c = c + this(i).length;
				end % for
			end % if
		end % matlabCode

		function fortranCode(this, name, fid)
		%FORTRANCODE Convert expression tree to fortran code.

			% Check input arguments
			if nargin == 2; fid = 1; end % if

			% Determine size of expression vector
			m = sum([this.length]);

			% Initialize index counter
			c = 1;

			for i = 1:numel(this)
				% Write FORTRAN 90 code
				str = fortranCode_(this(i));

				% Break line into pieces to avoid column overflow (f90 has 132 limit)
				for l = 72:72:2*length(str)
					if l+1 >= length(str); break; end % if
					str = [str(1:l) '&\n&' str((l+1):end)];
				end % for

				if strcmp(str, '0.0')
					% Do nothing, should already be preallocated to zero
				else
					% Otherwise we can set its value
					if this(i).length == 1
						% Scalar index
						fprintf(fid, '\t%s(%d) = %s;\n', name, c, str);
					else
						% Vector index
						fprintf(fid, '\t%s(%d:%d) = %s;\n', ...
							name, c, c + this(i).length - 1, str);
					end % if
				end % if

				% Advance index counter
				c = c + this(i).length;
			end % for
		end % fortranCode

		% ELEMENTWISE LOGICAL OPERATIONS --------------------------------------
		function bool = eq(a, b)
			% Check equality
			bool = isequal(a,b);
		end % eq

		% BINARY ELEMENTWISE OPERATIONS ---------------------------------------
		function c = plus(a, b)
			c = binaryHelper(@PlusBinaryOperatorNode, a, b);
		end % plus

		function c = minus(a, b)
			c = binaryHelper(@MinusBinaryOperatorNode, a, b);
		end % minus

		function c = times(a, b)
			c = binaryHelper(@TimesBinaryOperatorNode, a, b);
		end % times

		function c = rdivide(a, b)
			c = binaryHelper(@DivideBinaryOperatorNode, a, b);
		end % rdivide

		function c = power(a, b)
			c = binaryHelper(@PowerBinaryOperatorNode, a, b);
		end % power

		% BINARY MATRIX OPERATIONS --------------------------------------------
		function c = mtimes(a, b)
		%MTIMES Matrix multiplication.

			% Determine object dimensions
			[ai, aj] = size(a);
			[bi, bj] = size(b);

			% Check for scalar multiplication
			if numel(a) == 1 || numel(b) == 1
				c = times(a, b);

			% Check dimensions for matrix multiplication
			elseif aj == bi
				% Initialize empty object for result
				c(1:ai,1:bj) = ConstantNode(0);

				% Loop through each element in object array
				for i = 1:ai
					for j = 1:bj
						for k = 1:bi
							c(i,j) = PlusBinaryOperatorNode(...
								c(i,j), TimesBinaryOperatorNode(a(i,k), b(k,j)));
						end % for
					end % for
				end % for

			else
				error('coalesce:symbolic:ExpressionNode:mtimes', ...
					'Object array dimensions must agree.');
			end % if
		end % mtimes

		function c = mrdivide(a, b)
		%MRDIVIDE Matrix division.

			% Check for scalar division
			if numel(b) == 1
				c = rdivide(a, b);
			else
				error('Matrix division not supported.');
			end % if
		end % mrdivide

		function c = mpower(a, b)
		%MPOWER Matrix power.

			% Check for scalar powers
			if numel(a) == 1 && numel(b) == 1
				c = power(a, b);
			else
				error('Matrix powers not supported.');
			end % if
		end % mpower

		% UNARY ELEMENTWISE OPERATIONS ----------------------------------------
		function b = uplus(a)
			b = unaryHelper(@PlusUnaryOperatorNode, a);
		end % uplus

		function b = uminus(a)
			b = unaryHelper(@MinusUnaryOperatorNode, a);
		end % uminus

		% UNARY MATRIX OPERATIONS ---------------------------------------------
		% TODO: inv
		% TODO: eig

		% TRIGONOMETRIC OPERATIONS --------------------------------------------
		function b = cos(a)
			b = unaryHelper(@CosUnaryOperatorNode, a);
		end % cos

		function b = sin(a)
			b = unaryHelper(@SinUnaryOperatorNode, a);
		end % sin

		function b = tan(a)
			b = unaryHelper(@TanUnaryOperatorNode, a);
		end % tan

		% function b = sec(a)
		% 	b = unaryHelper(@SecUnaryOperatorNode, a);
		% end % sec
		%
		% function b = csc(a)
		% 	b = unaryHelper(@CscUnaryOperatorNode, a);
		% end % csc
		%
		% function b = cot(a)
		% 	b = unaryHelper(@CotUnaryOperatorNode, a);
		% end % cot
		%
		% % INVERSE TRIGONOMETRIC OPERATIONS ------------------------------------
		% function b = acos(a)
		% 	b = unaryHelper(@AcosUnaryOperatorNode, a);
		% end % acos
		%
		% function b = asin(a)
		% 	b = unaryHelper(@AsinUnaryOperatorNode, a);
		% end % asin
		%
		% function b = atan(a)
		% 	b = unaryHelper(@AtanUnaryOperatorNode, a);
		% end % atan
		%
		% function b = asec(a)
		% 	b = unaryHelper(@AsecUnaryOperatorNode, a);
		% end % asec
		%
		% function b = acsc(a)
		% 	b = unaryHelper(@AcscUnaryOperatorNode, a);
		% end % acsc
		%
		% function b = acot(a)
		% 	b = unaryHelper(@AcotUnaryOperatorNode, a);
		% end % acot
		%
		% % HYPERBOLIC TRIGONOMETRIC OPERATIONS ---------------------------------
		% function b = cosh(a)
		% 	b = unaryHelper(@CoshUnaryOperatorNode, a);
		% end % cosh
		%
		% function b = sinh(a)
		% 	b = unaryHelper(@SinhUnaryOperatorNode, a);
		% end % sinh
		%
		% function b = tanh(a)
		% 	b = unaryHelper(@TanhUnaryOperatorNode, a);
		% end % tanh
		%
		% function b = sech(a)
		% 	b = unaryHelper(@SechUnaryOperatorNode, a);
		% end % sech
		%
		% function b = csch(a)
		% 	b = unaryHelper(@CschUnaryOperatorNode, a);
		% end % csch
		%
		% function b = coth(a)
		% 	b = unaryHelper(@CothUnaryOperatorNode, a);
		% end % coth
		%
		% % INVERSE HYPERBOLIC TRIGONOMETRIC OPERATIONS -------------------------
		% function b = acosh(a)
		% 	b = unaryHelper(@AcoshUnaryOperatorNode, a);
		% end % acosh
		%
		% function b = asinh(a)
		% 	b = unaryHelper(@AsinhUnaryOperatorNode, a);
		% end % asinh
		%
		% function b = atanh(a)
		% 	b = unaryHelper(@AtanhUnaryOperatorNode, a);
		% end % atanh
		%
		% function b = asech(a)
		% 	b = unaryHelper(@AsechUnaryOperatorNode, a);
		% end % asech
		%
		% function b = acsch(a)
		% 	b = unaryHelper(@AcschUnaryOperatorNode, a);
		% end % acsch
		%
		% function b = acoth(a)
		% 	b = unaryHelper(@AcothUnaryOperatorNode, a);
		% end % acoth

		% SPECIAL OPERATIONS --------------------------------------------------
		function b = sqrt(a)
			b = unaryHelper(@SqrtUnaryOperatorNode, a);
		end % sqrt

		function b = log(a)
			b = unaryHelper(@LogUnaryOperatorNode, a);
		end % log

		% function b = log10(a)
		% 	b = unaryHelper(@Log10UnaryOperatorNode, a);
		% end % log10
		%
		% function b = log2(a)
		% 	b = unaryHelper(@Log2UnaryOperatorNode, a);
		% end % log2

		function b = exp(a)
			b = unaryHelper(@ExpUnaryOperatorNode, a);
		end % exp

		function b = abs(a)
			b = unaryHelper(@AbsUnaryOperatorNode, a);
		end % abs

		function b = sign(a)
			b = unaryHelper(@SignUnaryOperatorNode, a);
		end % sign

		% function b = dirac(a)
		% 	b = unaryHelper(@DiracUnaryOperatorNode, a);
		% end % dirac

		% ELEMENTARY OPERATIONS -------------------------------------------------
		function b = sum(a)
			b = unaryHelper(@SumUnaryOperatorNode, a);
		end % sum

		function c = trapz(a, b)
			c = b*sum((ind(a,1:a.length-1) + ind(a,2:a.length))/2);
			% c = unaryHelper(@SumUnaryOperatorNode, a);
		end % trapz

	end % methods
end % classdef

% HELPER FUNCTIONS ========================================================
function b = unaryHelper(operatorFcn, a)
%UNARYHELPER Helper function for unary elementwise operations.

	% Check dimensions of object arrays
	aSize = size(a);

	% Loop through and perform element wise operation
	for col = 1:aSize(2)
		for row = 1:aSize(1)
			b(row,col,:) = operatorFcn(a(row, col));
		end % for
	end % for
end % unaryHelper

function c = binaryHelper(operatorFcn, a, b)
%BINARYHELPER Helper function for binary elementwise operations.

	% Check dimensions of object arrays
	aSize = size(a);
	bSize = size(b);

	if all(aSize == bSize) || all(aSize == 1) || all(bSize == 1)
		% Loop through and perform element wise operation
		for col = 1:max(aSize(2), bSize(2))
			for row = 1:max(aSize(1), bSize(1))
				c(row,col,:) = operatorFcn(...
					a(min(row, aSize(1)), min(col, aSize(2))), ...
					b(min(row, bSize(1)), min(col, bSize(2))));
			end % for
		end % for
	else
		error('Dimensions do not match.');
	end % if
end % binaryHelper

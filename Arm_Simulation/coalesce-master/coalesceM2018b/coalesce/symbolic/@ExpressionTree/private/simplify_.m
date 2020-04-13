function b = simplify_(a)
%SIMPLIFY

  % Check if expression tree has already been simplified
  if a.isSimple; b = a; return; end % if

  % Check if lhs can be simplified
  if a.isUnary || a.isBinary
    if ~a.left.isSimple
      a.left = simplify_(a.left);
    end % if
  end % if

  % Check if rhs can be simplified
  if a.isBinary
    if ~a.right.isSimple
      a.right = simplify_(a.right);
    end % if
  end % if

  % Check for unary operations that can be evaluated
  if a.isUnary
    if strcmp(a.left.type, 'CONSTANT')
      b = ExpressionTree(eval(a));
      return;
    end % if
  % Check for binary operations that can be evaluated
  elseif a.isBinary
    if strcmp(a.left.type, 'CONSTANT') && strcmp(a.right.type, 'CONSTANT')
      b = ExpressionTree(eval(a));
      return;
    end % if
  end % if

  % Set output in case no rules apply
  b = a;

  % Simplification rules
  switch a.type
  case 'PLUS'
    % Simplification rule (0 + x = x)
    if isequal(a.left, ExpressionTree(0))
      b = a.right;
    % Simplification rule (x + 0 = x)
    elseif isequal(a.right, ExpressionTree(0))
      b = a.left;
    % Simplification rule (x + x = 2*x)
    elseif isequal(a.left, a.right)
      b = 2.*a.left;
    % Simplification rule (a + (- b) = a - b)
    elseif strcmp(a.right.type, 'UMINUS')
      b = a.left - a.right.left;
    % % Simplification rule (a + 2*a = 3*a)
    % elseif strcmp(a.right.type, 'TIMES') && (a.right.right == a.left)
    %   b = (1 + a.right.left).*a.left;;
    end % if

  case 'MINUS'
    % Simplification rule (0 - x = -x)
    if isequal(a.left, ExpressionTree(0))
      b = -a.right;
    % Simplification rule (x - 0 = x)
    elseif isequal(a.right, ExpressionTree(0))
      b = a.left;
    % Simplification rule (x - x = 0)
    elseif isequal(a.left, a.right)
      b = ExpressionTree(0);
    % Simplification rule (a - (- b) = a + b)
    elseif strcmp(a.right.type, 'UMINUS')
      b = a.left + a.right.left;
    end % if

  case 'TIMES'
    % Simplification rule (1 * x = x)
    if isequal(a.left, ExpressionTree(1))
      b = a.right;
    % Simplification rule (x * 1 = x)
    elseif isequal(a.right, ExpressionTree(1))
      b = a.left;
    % Simplification rule (-1 * x = -x)
    elseif isequal(a.left, ExpressionTree(-1))
      b = -a.right;
    % Simplification rule (x * -1 = -x)
    elseif isequal(a.right, ExpressionTree(-1))
      b = -a.left;
    % Simplification rule (0 * x = 0 and x * 0 = 0)
    elseif isequal(a.left, ExpressionTree(0)) || isequal(a.right, ExpressionTree(0))
      b = ExpressionTree(0);
    % Simplification rule (x * x = x^2)
    elseif isequal(a.left, a.right)
      b = a.left.^2;
    end % if

  case 'RDIVIDE'
    % Simplification rule (0 / x = 0)
    if isequal(a.left, ExpressionTree(0))
      b = ExpressionTree(0);
    % Simplification rule (x / x = 1)
    elseif isequal(a.left, a.right)
      b = ExpressionTree(1);
    % Simplification rule (1 / x = x^-1)
    elseif isequal(a.left, ExpressionTree(1))
      b = a.right.^-1;
    end % if

  case 'POWER'
    % Simplification rule (x ^ 0 = 1)
    if isequal(a.right, ExpressionTree(0))
      b = ExpressionTree(1);
    % Simplification rule (x ^ 1 = x)
    elseif isequal(a.right, ExpressionTree(1))
      b = a.left;
    % Simplification rule (0 ^ x = 0)
    elseif isequal(a.left, ExpressionTree(0))
      b = ExpressionTree(0);
    % Simplification rule (1 ^ x = 1)
    elseif isequal(a.left, ExpressionTree(1))
      b = ExpressionTree(1);
    end % if

  case 'UPLUS'
    % Simplification rule (+x = x)
    b = a.left;

  case 'UMINUS'
  case 'COS'
  case 'SIN'
  case 'TAN'
  case 'SQRT'
  case 'EXP'
  case 'LOG'

  otherwise
    error(['Not a valid operator (' a.type ').']);
  end % switch

  % Flip simplification flag
  b.isSimple = true;
end % simplify

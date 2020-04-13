function b = eval(a)
  if isa(a.left, 'ExpressionTree')
    a.left = eval(a.left);
  end % if

  if isa(a.right, 'ExpressionTree')
    a.right = eval(a.right);
  end % if

  switch a.type
  case 'CONSTANT'
    b = a.left;

  case 'VARIABLE'
    error('Variable cannot be evaluated.');

  case 'PLUS'
    b = a.left + a.right;

  case 'MINUS'
    b = a.left - a.right;

  case 'TIMES'
    b = a.left.*a.right;

  case 'RDIVIDE'
    b = a.left./a.right;

  case 'POWER'
    b = a.left.^a.right;

  case 'UPLUS'
    b = a.left;

  case 'UMINUS'
    b = -a.left;

  case 'COS'
    b = cos(a.left);

  case 'SIN'
    b = sin(a.left);

  case 'TAN'
    b = tan(a.left);

  case 'SQRT'
    b = sqrt(a.left);

  case 'EXP'
    b = exp(a.left);

  case 'LOG'
    b = log(a.left);

  otherwise
    error(['Not a valid operator (' a.type ').']);
  end % switch
end % eval

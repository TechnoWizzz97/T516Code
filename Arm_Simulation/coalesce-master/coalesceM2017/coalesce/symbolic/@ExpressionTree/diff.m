function c = diff(a, b)

  if a.isUnary
    l = a.left; dl = diff(l, b);
  elseif a.isBinary
    l = a.left; dl = diff(l, b);
    r = a.right; dr = diff(r, b);
  end % if

  switch a.type
  case 'CONSTANT'
    c = ExpressionTree(0);

  case 'VARIABLE'
    c = ExpressionTree(double(isequal(a, b)));

  case 'PLUS'
    c = dl + dr;

  case 'MINUS'
    c = dl - dr;

  case 'TIMES'
    if isequal(dl, ExpressionTree(0)) && isequal(dr, ExpressionTree(0))
      c = ExpressionTree(0);
    elseif isequal(dl, ExpressionTree(0))
      c = l.*dr;
    elseif isequal(dr, ExpressionTree(0))
      c = dl.*r;
    else
      c = dl.*r + l.*dr;
    end % if

  case 'RDIVIDE'
    if isequal(dl, ExpressionTree(0)) && isequal(dr, ExpressionTree(0))
      c = ExpressionTree(0);
    elseif isequal(dl, ExpressionTree(0))
      c = -dr.*l./r.^2;
    elseif isequal(dr, ExpressionTree(0))
      c = dl./r;
    else
      c = (dl.*r - l.*dr)./r.^2;
    end % if

  case 'POWER'
    if isequal(dl, ExpressionTree(0)) && isequal(dr, ExpressionTree(0))
      c = ExpressionTree(0);
    elseif isequal(dr, ExpressionTree(0))
      c = r.*dl.*(l.^(r - 1));
    else
      c = l.^r.*(dl.*r./l + dr.*log(l));
    end % if

  case 'UPLUS'
    c = dl;

  case 'UMINUS'
    c = -dl;

  case 'COS'
    c = -sin(l).*dl;

  case 'SIN'
    c = cos(l).*dl;

  case 'TAN'
    c = (tan(l).^2 + 1).*dl;

  case 'SQRT'
    c = dl./2./sqrt(l);

  case 'EXP'
    c = exp(l).*dl;

  case 'LOG'
    c = dl./l;

  otherwise
    error(['Not a valid operator (' a.type ').']);
  end
end % diff

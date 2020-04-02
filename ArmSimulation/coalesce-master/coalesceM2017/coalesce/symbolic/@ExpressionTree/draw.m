function draw(a, n)

  if nargin == 1
    n = 0;
  end % if

  space = repmat('| ', 1, n);

  switch a.type
  case {'CONSTANT','VARIABLE'}
    fprintf('  %s+ %s - %s\n', space, a.type, char(a));

  case {'PLUS','MINUS','TIMES','RDIVIDE','POWER'}
    fprintf('  %s+ - %s\n', space, a.type);
    draw(a.left, n+1);
    draw(a.right, n+1);

  case {'UPLUS','UMINUS','COS','SIN','TAN','SQRT','EXP','LOG'}
    fprintf('  %s+ %s\n', space, a.type);
    draw(a.left, n+1);

  otherwise
    error(['Not a valid operator (' a.type ').']);
  end
end % char

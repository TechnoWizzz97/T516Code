function b = char(a)

  switch a.type
  case 'CONSTANT'
    % Convert numeric value to string keeping double precision
    str = sprintf('%.*f', ceil(-log10(eps(a.left))), a.left);

    % Remove trailing zeros
    tmp = regexp(str, '^0+(?!\.)|(?<!\.)0+$', 'split');
    b = tmp{1};

  case 'VARIABLE'
    b = a.left;

  case 'PLUS'
    b = ['(' char(a.left) '+' char(a.right) ')'];

  case 'MINUS'
    b = ['(' char(a.left) '-' char(a.right) ')'];

  case 'TIMES'
    b = ['(' char(a.left) '.*' char(a.right) ')'];

  case 'RDIVIDE'
    b = ['(' char(a.left) './' char(a.right) ')'];

  case 'POWER'
    b = ['(' char(a.left) '.^' char(a.right) ')'];

  case 'UPLUS'
    b = ['(' char(a.left) ')'];

  case 'UMINUS'
    b = ['(-' char(a.left) ')'];

  case 'COS'
    b = ['cos(' char(a.left) ')'];

  case 'SIN'
    b = ['sin(' char(a.left) ')'];

  case 'TAN'
    b = ['tan(' char(a.left) ')'];

  case 'SQRT'
    b = ['sqrt(' char(a.left) ')'];

  case 'EXP'
    b = ['exp(' char(a.left) ')'];

  case 'LOG'
    b = ['log(' char(a.left) ')'];

  otherwise
    error(['Not a valid operator (' a.type ').']);
  end
end % char

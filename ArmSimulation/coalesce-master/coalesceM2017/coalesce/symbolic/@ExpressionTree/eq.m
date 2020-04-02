function bool = eq(a,b)

  if isa(a, 'double')
    a = ExpressionTree(a);
  else
    a = simplify_(a);
  end % if

  if isa(b, 'double')
    b = ExpressionTree(b);
  else
    b = simplify_(b);
  end % if

  bool = isequal(a, b);
end

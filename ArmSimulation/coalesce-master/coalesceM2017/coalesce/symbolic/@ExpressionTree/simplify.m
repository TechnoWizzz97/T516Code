function a = simplify(a)
  for i = 1:numel(a)
    a(i) = simplify_(a(i));
  end % for
end % simplify

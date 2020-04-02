function obj = unaryHelper(type, left)
%UNARYHELPER Helper function for unary elementwise operations.

  % Check dimensions of object arrays
  lSize = size(left);

  % Loop through and perform element wise operation
  for col = 1:lSize(2)
    for row = 1:lSize(1)
      % Check for and convert doubles
      if isa(left, 'double')
        left = ExpressionTree(left);
      end % if

      obj(row,col) = ExpressionTree(type, left(row,col));
    end % for
  end % for
end % unaryHelper

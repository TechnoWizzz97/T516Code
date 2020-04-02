function obj = binaryHelper(type, left, right)
%BINARYHELPER Helper function for binary elementwise operations.

  % Check dimensions of object arrays
  lSize = size(left);
  rSize = size(right);

  % Make sure dimension are consistent
  if all(lSize == rSize) || all(lSize == 1) || all(rSize == 1)
    % Loop through and perform element wise operation
    for col = 1:max(lSize(2), rSize(2))
      for row = 1:max(lSize(1), rSize(1))
        % Check for and convert doubles
        if isa(left, 'double')
          left = ExpressionTree(left);
        end % if

        % Check for and convert doubles
        if isa(right, 'double')
          right = ExpressionTree(right);
        end % if

        obj(row,col) = ExpressionTree(type, ...
          left(min(row, lSize(1)), min(col, lSize(2))), ...
          right(min(row, rSize(1)), min(col, rSize(2))));
      end % for
    end % for
  else
    error('Dimensions do not match.');
  end % if
end % binaryHelper

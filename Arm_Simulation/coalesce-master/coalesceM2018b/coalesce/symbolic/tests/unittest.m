% Construct constant and variables
c = ExpressionTree(1);
a = ExpressionTree('a');
x = ExpressionTree('x');

% Constant
disp(diff(c, c) == 0)
disp(diff(c, x) == 0)

% Variable
disp(diff(x, c) == 0)
disp(diff(x, x) == 1)

% Plus
disp(diff(x + x, x) == 2)

% Minus
disp(diff(x - x, x) == 0)

% Times
disp(diff(2.*x, x) == 2)
disp(diff(a.*x, x) == a)
disp(diff(x.*2, x) == 2)
disp(diff(x.*a, x) == a)

% Divide
disp(diff(2./x, x) == -2./x.^2)
disp(diff(a./x, x) == -a./x.^2)
disp(diff(x./2, x) == 1./2)
disp(diff(x./a, x) == 1./a) % Fails because can't simplify a/a^2 to 1/a

% Power
disp(diff(x.^2, x) == 2.*x)
disp(diff(2.^x, x) == 0.6931471805599453.*2.^x)

% Trig
disp(diff(sin(x), x) == cos(x))
disp(diff(cos(x), x) == -sin(x))
disp(diff(tan(x), x) == 1 + tan(x).^2) % Fails because of order

% Other
disp(diff(exp(x), x) == exp(x))
disp(diff(exp(-x), x) == -exp(-x))
disp(diff(log(x), x) == 1./x)

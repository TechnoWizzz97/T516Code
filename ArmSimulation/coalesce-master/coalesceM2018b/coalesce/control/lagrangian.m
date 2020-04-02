function [M, f, x, dx, fcn] = lagrangian(L, D, Q, q, u, p)
%LAGRANGIAN Compute equations of motion using Lagrangian method.
%
% Syntax:
%   [M, f, x, dx, fcn] = lagrangian(L, D, Q, q, u, p)
%
% Description:
%   Computes the equation of motion for a system using the Lagrangian
%   method.
%
% Input Arguments:
%   L - Symbolic Lagrangian equation
%   D - Dissipation equation
%   Q - Generalized force equations
%   q - Generalized coordinates
%   u - Control inputs
%   p - Parameters
%
% Output Arguments:
%   M - Inertia matrix
%   f - All other terms
%   x - State vector
%   dx - Differential equations
%   fcn - Dynamics function handle
%   names - Dynamics state names
%
% Copyright 2013-2014 Mikhail S. Jones

  fprintf('Solving  Lagrange''s equation... \n');

  % Time symbolic variable
  syms t;

  % Create substitution cell arrays for derivatives of the generalized
  % coordinates
  for iq = numel(q):-1:1
    qNew(3*iq) = sym(['DD' char(q(iq))]);
    qNew(3*iq-1) = sym(['D' char(q(iq))]);
    qNew(3*iq-2) = q(iq);

    qOld(3*iq) = str2sym(['diff(' char(q(iq)) '(t), t, t)']);
    qOld(3*iq-1) = str2sym(['diff(' char(q(iq)) '(t), t)']);
    qOld(3*iq-2) = str2sym([char(q(iq)) '(t)']);
  end % for

  sL = subs(L, qOld, qNew);
  sD = subs(D, qOld, qNew);
  sQ = subs(Q, qOld, qNew);

  % Loop through generalized coordinates
  for iq = numel(q):-1:1
    % Time derivative of momentum [d/dt*(dT/dDq) = m*DDq = F]
    dLdDq = diff(sL, qNew(iq*3-1));
    dLdDqdt = diff(subs(dLdDq, qNew, qOld), t);

    % Potential energy derivative [-dV/dq = F]
    dLdq = diff(sL, qNew(iq*3-2));

    % Dissipation force derivative [dD/dDq = F]
    dDdDq = diff(sD, qNew(iq*3-1));

    % Virtual work derivative [dQ/dq = F]
    dQdq = diff(sQ, qNew(iq*3-2));

    % Lagrange's equation [d/dt(dL/dDq) - dL/dq + dD/dDq = F*dr/dq]
    Leqn(iq) = subs(dLdDqdt - dLdq + dDdDq - dQdq, qOld, qNew);
  end % for

  % Substitute control inputs
  for iu = numel(u):-1:1
    uOld(iu) = str2sym([char(u(iu)) '(t)']);
    uNew(iu) = u(iu);
  end % for
  Leqn = subs(Leqn, uOld, uNew);

  % Put in matrix form
  fprintf('Putting in general matrix form... \n');
  [M, f] = equationsToMatrix(Leqn, qNew(3:3:end));

  if nargout > 2
    fprintf('Order reduction... \n');
    x = [qNew(1:3:end) qNew(2:3:end)].';
  end % if

  if nargout > 3
    dx = [qNew(2:3:end).'; M\f];

    fprintf('Simplifying... 0%%'); str = '0%';
    for i = 1:numel(dx)
      backspace = repmat(sprintf('\b'), 1, numel(str));
      str = [num2str(round(100*i/numel(dx))) '%'];
      fprintf('%s%s', backspace, str);
      dx(i) = simplify(dx(i));
    end % for
    fprintf('\b\b\b\b\n');
  end % if

  if nargout > 4
    % Convert to function handle
    fprintf('Constructing dynamics function handle... \n');
    fcn = matlabFunction(dx, 'vars', {x, u, p});
  end % if
end % lagrangian

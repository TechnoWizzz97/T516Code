%QUADROTORSYSTEM Quadrotor first order system model.
%
% Copyright 2014 Mikhail S. Jones

classdef QuadrotorSystem < FirstOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		g = 9.81 % Gravity
		m = 1 % Mass
		L = 0.25 % Rotor arm length
		J = 0.1 % Moment of inertia
		uLim = 10 % Prop thrust limit
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = QuadrotorSystem
		%QUADROTORSYSTEM Quadrotor first order system constructor.

			% Call superclass constructor
			obj = obj@FirstOrderSystem(...
				{'x', 'z', 'theta', 'Dx', 'Dz', 'Dtheta'}, ...
				{'u1', 'u2'}, ...
				{'Flight'});

			% Set input bounds
			obj.inputLowerBounds = 0;
			obj.inputUpperBounds = obj.uLim;
		end % QuadrotorSystem

		function xdot = stateEquation(obj, t, x, u, m)
		%STATEEQUATION The system state equation.

			% State equation
			xdot(6,1) = (u(2) - u(1))*obj.L/obj.J;
			xdot(5,1) = -obj.g + (u(1) + u(2))*cos(x(3))/obj.m;
			xdot(4,1) = -(u(1) + u(2))*sin(x(3))/obj.m;
			xdot(3,1) = x(6);
			xdot(2,1) = x(5);
			xdot(1,1) = x(4);
		end % stateEquation
	end % methods
end % classdef

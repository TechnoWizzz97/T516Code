%ACROBOTSYSTEM Acrobot system model.
%
% Copyright 2014 Mikhail S. Jones

classdef AcrobotSystem < SecondOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		g = 9.81 % Gravity
		m1 = 0.5 % Link 1 mass
		l1 = 0.5 % Link 1 total length
		r1 = 0.25 % Link 1 distance to center of mass
		I1 = 0.5*0.5^2/12 % Link 1 inertia at center of mass
		b1 = 0.1 % Link 1 damping constant
		m2 = 0.5 % Link 2 mass
		l2 = 0.5 % Link 2 total length
		r2 = 0.5 % Link 2 distance to center of mass
		I2 = 0.5*1^2/12 % Link 2 inertia at center of mass
		b2 = 0.1 % Link 2 damping
		tauLim = 5 % Torque limit
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = AcrobotSystem
		%ACROBOTSYSTEM Acrobot system model constructor.

			% Call superclass constructor
			obj = obj@SecondOrderSystem(...
				{'theta1', 'theta2'}, ...
				{'tau'}, ...
				{''});

			% Set input bounds
			obj.inputLowerBounds = -obj.tauLim;
			obj.inputUpperBounds = obj.tauLim;
		end % AcrobotSystem

		function [M, f] = secondOrderStateEquation(obj, t, x, xdot, u, m)
		%SECONDORDERSTATEEQUATION The system state equation.

			% Inertia matrix
			M(2,2) = 0*x(1);
			M(2,2) = obj.m2*obj.r2^2 + obj.I2;
			M(2,1) = obj.l1*obj.m2*obj.r2*cos(x(1) - x(2));
			M(1,2) = M(2,1);
			M(1,1) = obj.m2*obj.l1^2 + obj.m1*obj.r1^2 + obj.I1;

			% Other terms
			f(2,1) = obj.l1*obj.m2*obj.r2*sin(x(1) - x(2))*xdot(1)^2 + obj.b2*xdot(1) + u(1) - xdot(2)*obj.b2 - obj.g*obj.m2*obj.r2*cos(x(2));
			f(1,1) = - obj.l1*obj.m2*obj.r2*sin(x(1) - x(2))*xdot(2)^2 + obj.b2*xdot(2) - u(1) - xdot(1)*obj.b1 - xdot(1)*obj.b2 - obj.g*obj.l1*obj.m2*cos(x(1)) - obj.g*obj.m1*obj.r1*cos(x(1));
		end % secondOrderStateEquation
	end % methods
end % classdef

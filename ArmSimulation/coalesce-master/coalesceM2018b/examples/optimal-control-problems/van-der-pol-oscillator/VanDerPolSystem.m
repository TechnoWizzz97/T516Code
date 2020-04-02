%VANDERPOLSYSTEM Van der Pol oscillator system model.
%
% Copyright 2014 Mikhail S. Jones

classdef VanDerPolSystem < SecondOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		mu = 1 % Change the stiffness of the ODE
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = VanDerPolSystem
		%VANDERPOLSYSTEM Van der Pol oscillator system model constructor.

			% Call superclass constructor
			obj = obj@SecondOrderSystem(...
				{'z'}, ...
				{'u'}, ...
				{''});
		end % VanDerPolSystem

		function [M, f] = secondOrderStateEquation(obj, t, x, xdot, u, m)
		%SECONDORDERSTATEEQUATION The system state equation.

			% Inertia matrix
			M = 1;

			% Other terms
			f = obj.mu*(1 - x.^2)*xdot - x + u;
		end % secondOrderStateEquation
	end % methods
end % classdef

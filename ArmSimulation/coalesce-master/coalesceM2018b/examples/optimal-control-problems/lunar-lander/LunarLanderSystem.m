%LUNARLANDERSYSTEM Lunar lander system model.
%
% Copyright 2014 Mikhail S. Jones

classdef LunarLanderSystem < SecondOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		m = 1 % Mass
		g = 1.63 % Gravity on Earth's moon
		fLim = 5 % Thrust limit
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = LunarLanderSystem
		%LUNARLANDERSYSTEM Lunar lander system model constructor.

			% Call superclass constructor
			obj = obj@SecondOrderSystem(...
				{'z'}, ...
				{'f'}, ...
				{''});

			% Set input bounds
			obj.inputLowerBounds = 0;
			obj.inputUpperBounds = obj.fLim;
		end % LunarLanderSystem

		function [M, f] = secondOrderStateEquation(obj, t, x, xdot, u, m)
		%SECONDORDERSTATEEQUATION The system state equation.

			% Inertia matrix
			M = obj.m;

			% Other terms
			f = u - obj.g;
		end % secondOrderStateEquation
	end % methods
end % classdef

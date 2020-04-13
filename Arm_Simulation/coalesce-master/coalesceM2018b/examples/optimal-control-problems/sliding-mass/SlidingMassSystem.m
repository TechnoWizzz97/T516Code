%SLIDINGMASSSYSTEM Sliding mass second order system model.
%
% Copyright 2014 Mikhail S. Jones

classdef SlidingMassSystem < SecondOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		m = 1 % Mass
		b = 0 % Damping constant
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = SlidingMassSystem
		%SLIDINGMASSSYSTEM Construct sliding mass system model.

			% Call superclass constructor
			obj = obj@SecondOrderSystem(...
				{'x'}, ...
				{'f'}, ...
				{''});
		end % SlidingMassSystem

		function [M, f] = secondOrderStateEquation(obj, t, x, xdot, u, m)
		%SECONDORDERSTATEEQUATION The system state equation.

			% Inertia matrix
			M = obj.m;

			% Other terms
			f = u - obj.b*xdot;
		end % secondOrderStateEquation
	end % methods
end % classdef

%BRACHISTOCHRONESYSTEM Brachistochrone first order system model.
%
% Copyright 2014 Mikhail S. Jones

classdef BrachistochroneSystem < FirstOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		g = 9.81 % Gravity
		uLim = pi/2 % Control input limit
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = BrachistochroneSystem
		%BRACHISTOCHRONESYSTEM Brachistochrone first order system constructor.

			% Call superclass constructor
			obj = obj@FirstOrderSystem(...
				{'x', 'y', 'v'}, ...
				{'u'}, ...
				{''});

			% Set input bounds
			obj.inputLowerBounds = -obj.uLim;
			obj.inputUpperBounds = obj.uLim;
		end % BrachistochroneSystem

		function xdot = stateEquation(obj, t, x, u, m)
		%STATEEQUATION The system state equation.

			% State equation
			xdot(3,1) = obj.g*cos(u);
			xdot(2,1) = x(3)*cos(u);
			xdot(1,1) = x(3)*sin(u);
		end % stateEquation
	end % methods
end % classdef

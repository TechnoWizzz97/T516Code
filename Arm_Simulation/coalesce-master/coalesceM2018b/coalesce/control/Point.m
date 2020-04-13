%POINT Fixed point of a dynamic system.
%
% Copyright 2014 Mikhail S. Jones

classdef Point

	% PROTECTED PROPERTIES ==================================================
	properties (SetAccess = protected)
		states@double vector % Vector of state variables (x)
		inputs@double vector % Vector of input variables (u)
		modes@char vector % Dynamic mode (m)
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = Point(x, u, m)
		%POINT Dynamic system fixed point constructor.

			% Set object properties
			obj.states = x;
			obj.inputs = u;
			obj.modes = m;
		end % Point

		function [x, u, m] = unpack(obj)
		%UNPACK Unpack the point object structure for external use.

			x = obj.states;
			u = obj.inputs;
			m = obj.modes;
		end % unpack
	end % methods
end % classdef

%RESPONSE Time response of a dynamic system.
%
% Copyright 2014 Mikhail S. Jones

classdef Response

	% PROTECTED PROPERTIES ==================================================
	properties
		time@cell vector % Cell array of times (t)
		states@cell vector % Cell array of state variables (x)
		inputs@cell vector % Cell array of input variables (u)
		modes@cell vector % Cell array of dynamic modes (m)
		stateNames@cell vector
		inputNames@cell vector
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = Response(t, x, u, m, varargin)
		%RESPONSE Time response of a dynamic system.

			for im = 1:numel(t)
				nt = numel(t{im});

				obj.time{im} = reshape(t{im}, 1, nt);

				if size(x{im}, 2) ~= nt
					obj.states{im} = x{im}.';
				else
					obj.states{im} = x{im};
				end % if

				if size(u{im}, 2) ~= nt
					obj.inputs{im} = u{im}.';
				else
					obj.inputs{im} = u{im};
				end % if

				obj.modes{im} = m{im};
			end % for

			if nargin == 6
				[obj.stateNames, obj.inputNames] = varargin{:};
			end % if
		end % Response

		function [t, x, u, m] = unpack(obj)
		%UNPACK Unpack the response object structure for external use.

			t = obj.time;
			x = obj.states;
			u = obj.inputs;
			m = obj.modes;
		end % unpack

		function plot(obj)
		%PLOT Plot the state and input time histories.

			% Initialize figure
			figure;

			% Plot state histories
			subplot(2,1,1); hold on; grid on; box on;
			plot([obj.time{:}], [obj.states{:}], '.-');
			title('State Time Response');
			if ~isempty(obj.stateNames)
				legend(obj.stateNames, 'Location', 'NorthEastOutside');
			end % if

			% Plot input histories
			subplot(2,1,2); hold on; grid on; box on;
			plot([obj.time{:}], [obj.inputs{:}], '.-');
			title('Input Time Response');
			if ~isempty(obj.inputNames)
				legend(obj.inputNames, 'Location', 'NorthEastOutside');
			end % if
		end % plot

		function [x, u] = eval(obj, t)
		%EVAL Evaluate response object at given time.

			% TODO return point or response object
			x = interp1([obj.time{:}].', [obj.states{:}].', t).';
			u = interp1([obj.time{:}].', [obj.inputs{:}].', t).';
		end % eval
	end % methods
end % classdef

function response = simulate(obj, tSpan, x0, varargin)
%SIMULATE Simulate the hybrid first order input-output system.
%
% Required Input Arguments:
%		tSpan - (DOUBLE) Time span to simulate over
%		x0 - (DOUBLE) Initial state vector
%
% Optional Input Arguments:
%		InitialMode - (CHAR) Name of the initial dynamic mode
%		MaxModes - (DOUBLE) Maximum number of mode switches
%		Controller - (FUNCTION_HANDLE) Controller function
%
% Copyright 2014 Mikhail S. Jones

	% Number of states, inputs and modes in schedule
	nx = numel(obj.states);
	nu = numel(obj.inputs);

	% Parse input arguments
	parser = inputParser;
	parser.addRequired('timeSpan', ...
		@(x) validateattributes(x, ...
			{'double'}, {'vector'}));
	parser.addRequired('initialState', ...
		@(x) validateattributes(x, {'double'}, {'numel', nx}));
	parser.addOptional('initialMode', obj.modes{1}, ...
		@(x) ischar(validatestring(x, obj.modes)));
	parser.addOptional('maxModes', inf, ...
		@(x) validateattributes(x, {'double'}, {'scalar', 'positive'}));
	parser.addOptional('controller', @(t,x) 0, ...
		@(x) validateattributes(x, {'function_handle'}, {}));
	parser.addOptional('solver', 'ode45', ...
		@(x) ischar(validatestring(x, ...
			{'ode45', 'ode23', 'ode113', 'ode15s', 'ode23s', 'ode23t', 'ode23tb'})));
	parser.parse(tSpan, x0, varargin{:});
	opts = parser.Results;

	% Convert solver string to function handle
	opts.solver = str2func(opts.solver);

	% Initialize time, mode and counter
	t0 = tSpan(1); m0 = opts.initialMode; im = 0;

	% Wrap controller to respect control input bounds
	opts.controller = @(t,x) min(max(reshape(opts.controller(t,x), 1, []), obj.inputLowerBounds), obj.inputUpperBounds);

	% Simulate system
	% while im < opts.maxModes
		% Advance counter
		im = im + 1;

		% Define state equation function
		ode = @(t, x) obj.stateEquation(t, x, opts.controller(t, x), m0);

		% Define ODE options and event functions
		% options = odeset(...
		% 	'Events', @(t, x) events(t, x, m0));

		% Run ODE solver
		% [t{im}, x{im}, te, xe, ie] = opts.solver(ode, [t0 tSpan(end)], x0(:), options);
		[t{im}, x{im}] = opts.solver(ode, [t0 tSpan(end)], x0(:));
		m{im} = m0;

		% Check if an event was triggered
		% if ~isempty(ie)
		% 	% Determine correct jump map to use
		% 	[~, ~, modes, jumpFcn] = obj.guardSet(t{c}(end), x{c}(end,:)', m0);
		%
		% 	% Evaluate jump map to determine new state
		% 	x0 = jumpFcn{ie}(x{c}(end,:)');
		% 	m0 = modes{ie};
		% end % if

		% Set new initial time
		% t0 = t{im}(end);

		% Loop through time vector backwards to preallocate
		for it = numel(t{im}):-1:1
			% Compute control inputs
			u{im}(it,:) = opts.controller(t{im}(it), x{im}(it,:)');
		end % for
	% end % while

	% Construct time response object
	response = Response(t, x, u, m, obj.states, obj.inputs);

	% function [value, isTerminal, direction] = events(m, t, x)
	% %EVENTS Guard set wrapper for ODE events function.
	%
	% 	[value, direction] = obj.guardSet(m, t, x);
	% 	isTerminal = true;
	% end % events
end % simulate

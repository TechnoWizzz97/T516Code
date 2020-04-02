%SLIDINGMASSSCENE Creates a sliding mass scene object.
%
% Description:
%   Creates a sliding mass object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef SlidingMassScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	x0@double % Lower position limit
  	xf@double % Upper position limit

  	axes%@double
  	ground@StripedLine
  	mass@RoundedSquare

  	response@Response
	end % properties

  % PUBLIC METHODS ========================================================
	methods
		function obj = SlidingMassScene(sys, response)
		%SLIDINGMASSSCENE Sliding mass scene object constructor.

			% Call superclass constructor
			obj = obj@Scene(response.time{1});

			% Store system response
			obj.response = response;

			% Store parameters
			obj.x0 = min(response.states{1}(1,:));
			obj.xf = max(response.states{1}(1,:));
		end % SlidingMassScene

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle we are plotting to
			obj.axes = gca;

			% Define graphical sliding mass
			obj.mass = RoundedSquare(0.4, 0.2, 0.05);

			% Define left wall
			obj.ground(1) = StripedLine(obj.mass.height, 0.025);
			obj.ground(1).rotate(0, 0, pi/2);
			obj.ground(1).scale(-1, 1, 1);
			obj.ground(1).translate(obj.x0 - obj.mass.width/2, 0, 0);
			obj.ground(1).update;

			% Define ground
			obj.ground(2) = StripedLine(obj.xf - obj.x0 + obj.mass.width, 0.025);
			obj.ground(2).translate(obj.x0 - obj.mass.width/2, 0, 0);
			obj.ground(2).update;

			% Define right wall
			obj.ground(3) = StripedLine(obj.mass.height, 0.025);
			obj.ground(3).scale(-1, 1, 1);
			obj.ground(3).rotate(0, 0, pi/2);
			obj.ground(3).translate(obj.xf + obj.mass.width/2, obj.mass.height, 0);
			obj.ground(3).update;

			% Axes properties
			view(2); axis off;
		end % initialize

		function obj = update(obj, t)
		%UPDATE Update graphical objects.

			% Evaluate response at current time
			[x, u] = obj.response.eval(t);

			% Update mass model
			obj.mass.reset;
			obj.mass.translate(x(1) - obj.mass.width/2, 0, 0);
			obj.mass.update;

			% Set axes limits
			x0 = obj.x0 - 1;
			xf = obj.xf + 1;
			w = xf - x0;
			ylim(obj.axes, [-w, w]*720/1280/2);
			xlim(obj.axes, [x0, xf]);
		end % update
	end % methods
end % classdef

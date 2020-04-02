%LUNARLANDERSCENE Creates a lunar lander scene object.
%
% Description:
%   Creates a lunar lander object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef LunarLanderScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	h0@double % Lower height limit
  	hf@double % Upper height limit
  	uMax@double % Max thrust

  	axes%@double
  	ground@StripedLine
  	lander@RoundedSquare
  	thruster@Triangle

  	response@Response
	end % properties

  % PUBLIC METHODS ========================================================
	methods
		function obj = LunarLanderScene(sys, response)
		%LUNARLANDERSCENE Lunar lander scene constructor.

			% Call superclass constructor
			obj = obj@Scene(response.time{1});

			% Store system response
			obj.response = response;

			% Store parameters
			obj.h0 = response.states{1}(1,1);
			obj.hf = response.states{1}(end,1);
			obj.uMax = max(response.inputs{1});
		end % LunarLanderScene

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle we are plotting to
			obj.axes = gca;

			% Define graphical rocket
			obj.lander = RoundedSquare(0.4, 1.2, 0.1);

			% Define graphical thruster
			w = 0.5*obj.lander.width;
			obj.thruster = Triangle(w, 2*w, [1 0.4 0]);
			set(obj.thruster.handle, 'EdgeColor', 'none');

			% Define ground
			obj.ground = StripedLine(10, 0.125);
			obj.ground.translate(-obj.ground.width/2, 0, 0);
			obj.ground.update;

			% Axes properties
			view([0, 89.9]); axis off;
		end % initialize

		function obj = update(obj, t)
		%UPDATE Update graphical objects.

			% Evaluate response at current time
			[x, u] = obj.response.eval(t);

			% Normalize thruster
			u = u/obj.uMax;

			% Update lander model
			obj.lander.reset;
			obj.lander.translate(-obj.lander.width/2, x(1), 2e-4);
			obj.lander.update;

			% Update thruster model
			obj.thruster.reset;
			obj.thruster.scale(u, -u, u);
			obj.thruster.translate(-obj.thruster.width/2*u + randn/100, x(1), 1e-4);
			obj.thruster.update;

			% Set axes limits
			h0 = obj.h0 + 2;
			hf = obj.hf - 1;
			h = h0 - hf;
			ylim(obj.axes, [hf, h0]);
			xlim(obj.axes, [-h, h]*1280/720/2);
		end % update
	end % methods
end % classdef

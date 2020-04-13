%QUADROTORSCENE Creates a quadrotor scene object.
%
% Description:
%   Creates a quadrotor object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef QuadrotorScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	L@double % Rotor arm length

   	axes%@double
  	ground@StripedLine
		body@RoundedSquare

  	response@Response
	end % properties

  % PUBLIC METHODS ========================================================
	methods
		function obj = QuadrotorScene(sys, response)
		%QUADROTORSCENE Quadrotor scene constructor.

			% Call superclass constructor
			obj = obj@Scene(response.time{1});

			% Store system response
			obj.response = response;

			% Store parameters
			obj.L = sys.L;
		end % QuadrotorScene

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle we are plotting to
			obj.axes = gca;

			% Define graphical pendulum on cart
			obj.body = RoundedSquare(obj.L, obj.L/5, obj.L/10, 'r');
			% obj.u1 = quiver3(0, 0, 0, 0, 0, 0, L/obj.uLim, 'm');
			% obj.u2 = quiver3(0, 0, 0, 0, 0, 0, L/obj.uLim, 'm');

			% Define ground
			obj.ground = StripedLine(25, 0.04);
			obj.ground.translate(-obj.ground.width/2, -obj.body.height/2, 0);
			obj.ground.update;

			% Axes properties
			view(0, 89.9); % Fixes issue where OpenGL will sometimes not draw objects
			axis off;
		end % initialize

		function obj = update(obj, t)
		%UPDATE Update graphical objects.

			% Evaluate response at current time
			[x, u] = obj.response.eval(t);

			% Update cart model
			obj.body.reset;
			obj.body.translate(- obj.body.width/2, - obj.body.height/2, 0);
			obj.body.rotate(0, 0, x(3));
			obj.body.translate(x(1), x(2), 1e-3);
			obj.body.update;

			% % Update prop 1 thrust
			% set(obj.u1, ...
			% 	'XData', s.x - L/2*cos(s.theta), ...
			% 	'YData', s.z - L/2*sin(s.theta), ...
			% 	'ZData', 0.01, ...
			% 	'UData', -s.u1*sin(s.theta), ...
			% 	'VData', s.u1*cos(s.theta));

			% % Update prop 2 thrust
			% set(obj.u2, ...
			% 	'XData', s.x + L/2*cos(s.theta), ...
			% 	'YData', s.z + L/2*sin(s.theta), ...
			% 	'ZData', 0.01, ...
			% 	'UData', -s.u2*sin(s.theta), ...
			% 	'VData', s.u2*cos(s.theta));

			% Set axes limits
			w = 3;
			xlim(obj.axes, x(1) + [-w, w]);
			ylim(obj.axes, x(2) + [-w, w]*720/1280);
		end % update
	end % methods
end % classdef

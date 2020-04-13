%ACROBOTSCENE Creates an acrobot scene object.
%
% Description:
%   Creates an acrobot object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef AcrobotScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	l1@double % Link 1 total length
  	l2@double % Link 2 total length

  	axes%@double
  	ground@StripedLine
  	base@RoundedSquare
  	link1@RoundedSquare
  	link2@RoundedSquare

  	response@Response
	end % properties

  % PUBLIC METHODS ========================================================
	methods
		function obj = AcrobotScene(sys, response)
		%ACROBOTSCENE Acrobot scene constructor.

			% Call superclass constructor
			obj = obj@Scene(response.time{1});

			% Store system response
			obj.response = response;

			% Store parameters
			obj.l1 = sys.l1;
			obj.l2 = sys.l2;
		end % AcrobotScene

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle
			obj.axes = gca;

			% Total length
			L = obj.l1 + obj.l2;

			% Define base
			obj.base = RoundedSquare(L/10, L/10, L/40);
			obj.base.translate(-obj.base.width/2, -obj.base.height/2, 0);
			obj.base.update;

			% Define ground
			obj.ground = StripedLine(L, L/40);
			obj.ground.translate(-obj.ground.width/2, -obj.base.height/2, 0);
			obj.ground.update;

			% Define links
			obj.link1 = RoundedSquare(obj.l1 + L/20, L/20, L/40, 'r');
			obj.link2 = RoundedSquare(obj.l2 + L/20, L/20, L/40, 'b');

			% Axes properties
			view(0, 89.9); % Fixes issue where OpenGL will sometimes not draw objects
			axis off;
		end % initialize

		function obj = update(obj, t)
		%UPDATE Update graphical objects.

			% Evaluate response at current time
			[x, u] = obj.response.eval(t);

			% Update link 1
			obj.link1.reset;
			obj.link1.translate(-obj.link1.height/2, -obj.link1.height/2, 0);
			obj.link1.rotate(0, 0, x(1));
			obj.link1.translate(0, 0, 1e-3);
			obj.link1.update;

			% Update link 2
			obj.link2.reset;
			obj.link2.translate(-obj.link2.height/2, -obj.link2.height/2, 0);
			obj.link2.rotate(0, 0, x(2));
			obj.link2.translate(obj.l1*cos(x(1)), obj.l1*sin(x(1)), 2e-3);
			obj.link2.update;

			% Set axes limits
			yLim = 1.1*(obj.l1 + obj.l2);
			xLim = yLim/720*1280;
			ylim(obj.axes, [-yLim, yLim]);
			xlim(obj.axes, [-xLim, xLim]);
		end % update
	end % methods
end % classdef

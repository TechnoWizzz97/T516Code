%HOPPERSCENE Creates an hopper scene object.
%
% Description:
%   Creates an hopper object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef TerrainHopperScene < Scene

  % PUBLIC PROPERTIES =====================================================
  properties
  	l1@double % Link 1 total length
  	l2@double % Link 2 total length

  	% axes@double
  	axes
  	groundleft@StripedLine
  	groundright@StripedLine

  	sand@RoundedSquare

  	mass@RoundedSquare
  	rod@RoundedSquare
  	foot@RoundedSquare

  	spring1@RoundedSquare
  	spring2@RoundedSquare
  	spring3@RoundedSquare
  	spring4@RoundedSquare
  	spring5@RoundedSquare
  	spring6@RoundedSquare

  	% response@Response
  	response
	end % properties

  % PUBLIC METHODS ========================================================
	methods
		function obj = TerrainHopperScene(response)
		%HOPPERSCENE Hopper scene constructor.

			% Call superclass constructor
			obj = obj@Scene(response.time{1});

			% Store system response
			obj.response = response;

			% Store parameters
			% obj.l1 = sys.l1;
			% obj.l2 = sys.l2;
		end % HopperScene

		function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.

			% Store the axes handle
			obj.axes = gca;

			% Total length
			% L = obj.l1 + obj.l2;
            
            GROUND_COLOR = [0.5 1 0.5]; %[0.9 0.9 0.6];
            JUMPER_COLOR = [0.85 0.85 0.85];
            MASS_COLOR = [0.85 0.85 0.85];

			% Define groundleft
			obj.groundleft = StripedLine(0.5, 0.5/80);
			obj.groundleft.translate(-0.5/2-0.275, -0.5/80 + 0.002, -1);
			obj.groundleft.update;

			obj.groundright = StripedLine(0.5, 0.5/80);
			obj.groundright.scale(-1, 1, 1);
			obj.groundright.translate(-0.5/2+0.775, -0.5/80 + 0.002, -1);
			obj.groundright.update;

			% Define sand
			obj.sand = RoundedSquare(0.05, 1, 0.001, GROUND_COLOR);
			obj.sand.translate(-0.05/2, -1-0.003, -2);
			obj.sand.update;

			% Define mass
			obj.mass = RoundedSquare(0.02, 0.02, 0.02/10,MASS_COLOR);
			obj.mass.translate(-0.02/2, -0.02/2, 0);
			obj.mass.update;

			% Define rod
% 			obj.rod = RoundedSquare(0.01, 0.05, 0.01/10,JUMPER_COLOR); % AHC [09/20/2018]: ICRA '19 fig gen. 
			obj.rod = RoundedSquare(0.01, 0.065, 0.01/10,JUMPER_COLOR);
			obj.rod.translate(-0.01/2, -0.05/2, 0);
			obj.rod.update;

			% Define foot
			obj.foot = RoundedSquare(0.03, 0.003, 0.005/10,JUMPER_COLOR);
			obj.foot.translate(-0.03/2, -0.003, 0);
			obj.foot.update;


			% Define spring 1
			obj.spring1 = RoundedSquare(0.002, 0.003, 0.001, 'k');
			obj.spring1.translate(-0.002/2, 0, -0.001);
			obj.spring1.update;

			% Define spring 2
			obj.spring2 = RoundedSquare(0.002, 0.02, 0.001, 'k');
			obj.spring2.translate(-0.002/2, -0.02/2, -0.05);
			obj.spring2.update;

			% Define spring 3
			obj.spring3 = RoundedSquare(0.005, 0.002, 0.001, 'k');
			obj.spring3.translate(-0.005/2, 0.002, 0);
			obj.spring3.update;


			% Define spring 4
			obj.spring4 = RoundedSquare(0.01, 0.002, 0.001, 'k');
			obj.spring4.translate(-0.01/2, 0.002, 0);
			obj.spring4.update;


			% Define spring 5
			obj.spring5 = RoundedSquare(0.01, 0.002, 0.001, 'k');
			obj.spring5.translate(-0.01/2, 0.002, 0);
			obj.spring5.update;

			% Define spring 6
			obj.spring6 = RoundedSquare(0.005, 0.002, 0.001, 'k');
			obj.spring6.translate(-0.005/2, 0, 0);
			obj.spring6.update;

			% obj.spring4 = RoundedSquare(0.002, 0.2, 0.001);
			% obj.spring4.translate(-0.002/2, -0.2/2, 0);
			% obj.spring4.update;

			% % Define links
			% obj.link1 = RoundedSquare(obj.l1 + L/20, L/20, L/40, 'r');
			% obj.link2 = RoundedSquare(obj.l2 + L/20, L/20, L/40, 'b');

			% Axes properties
			view(0, 89.9); % Fixes issue where OpenGL will sometimes not draw objects
			axis off;
		end % initialize

		function obj = update(obj, t)
		%UPDATE Update graphical objects.

			% Evaluate response at current time
			% [x, u] = obj.response.eval(t);
			response = obj.response;

			zm = interp1(response.time{1}, response.zm, t);
			zr = interp1(response.time{1}, response.zr, t);
			zf = interp1(response.time{1}, response.zf, t);
			zg = interp1(response.time{1}, response.zg, t);

			coils = 6;
			l = zr-zf-0.05/2 - 0.002;
			d = 0.005;
			s = atan2(l/coils,d);

			% Update sand
			obj.sand.reset;
			obj.sand.translate(-0.05/2, -1-0.002, -2);
			obj.sand.translate(0, zg, 0);
			obj.sand.update;

			% Update mass
			obj.mass.reset;
			obj.mass.translate(-0.02/2, -0.02/2, 0);
			obj.mass.translate(0, zm, 0);
			% obj.link1.rotate(0, 0, x(1));
			obj.mass.translate(0, 0, 2e-3);
			obj.mass.update;

			% Update rod
			obj.rod.reset;
			obj.rod.translate(-0.01/2, -0.05/2, 0);
			obj.rod.translate(0, zr, 0);
			% obj.link2.rotate(0, 0, x(2));
			obj.rod.translate(0, 0, 1e-3);
			obj.rod.update;

			% Update foot
			obj.foot.reset;
			obj.foot.translate(-0.03/2, -0.005, 0);
			obj.foot.translate(0, zf, 0);
			% obj.link2.rotate(0, 0, x(2));
			obj.foot.translate(0, 0, 3e-3);
			obj.foot.update;

			% Define spring 1
			obj.spring1.reset;
			obj.spring1.translate(-0.002/2, 0, -0.001);
			obj.spring1.translate(0, zf-0.0028, 0);
			obj.spring1.update;

			% Define spring 2
			obj.spring2.reset;
			obj.spring2.translate(-0.002/2, -0.02/2, -0.05);
			obj.spring2.scale(1, 4*l/0.2, 1);
			obj.spring2.translate(0, zf+0.89*l, 0);
			obj.spring2.update;


			% Define spring 3
			obj.spring3.reset;
			obj.spring3.translate(-0.005/2, 0, -0.002);
			obj.spring3.scale(sqrt((l/coils)^2+d^2)/0.005, 1, 1);
			obj.spring3.update;
			obj.spring3.rotate(0, 0, s);
			obj.spring3.translate(0.005/2, zf+0.01*l/coils, 0);
			obj.spring3.update;

			% Define spring 4
			obj.spring4.reset;
			obj.spring4.translate(-0.01/2, 0, -0);
			obj.spring4.scale(sqrt((l/coils)^2+(2*d)^2)/0.01, 1, 1);
			obj.spring3.update;
			obj.spring4.rotate(0, 0, -s);
			obj.spring4.translate(0-0.01*l, zf+1*l/coils, 0);
			obj.spring4.update;


			% Define spring 5
			obj.spring5.reset;
			obj.spring5.translate(-0.01/2, 0, -0.002);
			obj.spring5.scale(sqrt((l/coils)^2+(2*d)^2)/0.01, 1, 1);
			obj.spring3.update;
			obj.spring5.rotate(0, 0, s);
			obj.spring5.translate(0+0.02*l, zf+2.42*l/coils, 0);
			obj.spring5.update;

			% Define spring 6
			obj.spring6.reset;
			obj.spring6.translate(-0.005/2, 0, -0.002);
			obj.spring6.scale(sqrt((l/coils)^2+d^2)/0.005, 1, 1);
			obj.spring6.update;
			obj.spring6.rotate(0, 0, -s);
			obj.spring6.translate(0.005/2-0.04*l, zf+3.55*l/coils, 0);
			obj.spring6.update;


			% Set axes limits
			% yLim = 1.1*(obj.l1 + obj.l2);
			% xLim = yLim/720*1280;
			yLim = [-0.05, 0.2];
			ylim(obj.axes, [-0.05, 0.2]);
			% xlim(obj.axes, yLim/720*1280);
			xlim(obj.axes, [-0.2 0.2]);
		end % update
	end % methods
end % classdef

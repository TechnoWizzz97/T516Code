classdef DurusScene < Scene
%DURUSSCENE Creates a simple 2D Durus scene object.
%
% Description:
%   Creates a simple 2D Durus object using the Scene superclass.
%
% Copyright 2013-2014 Mikhail S. Jones

  properties
  	axes@double
  	ground@StripedLine
  	torso@RoundedSquare
  	sFoot@RoundedSquare
  	sShin@RoundedSquare
  	sThigh@RoundedSquare
  	nsFoot@RoundedSquare
  	nsShin@RoundedSquare
  	nsThigh@RoundedSquare

  	lThigh@double scalar = .42
  	lShin@double scalar = .32
  	lFoot@double scalar = .27
  	lTorso@double scalar = .5

  	response@Response
	end % properties

	methods
		function this = DurusScene(model, response)
		%DURUSSCENE Creates a simple 2D Durus scene object.
		%
		% Syntax:
		%   obj = DurusScene(model, response)
		%
		% Required Input Arguments:
		%		model - Durus system model
		%   response - Time response of Durus system
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Scene(response.time{1});

			% Store system response
			this.response = response;
		end % DurusScene

		function this = initialize(this)
		%INITIALIZE Initialize graphical objects.
		%

			% Define properties (torso radius and leg segment radius)
			r = 0.04;
			rr = 0.015;

			% Store the axes handle
			this.axes = gca;

			% Define ground
			this.ground = StripedLine(10, 0.025);
			this.ground.translate(-this.ground.width/2, 0, 0);
			this.ground.update;

			% Define graphical Durus
			this.torso = RoundedSquare(this.lTorso + 2*r, 2*r, r);
			this.sFoot = RoundedSquare(this.lFoot + 2*rr, 2*rr, rr);
			this.sShin = RoundedSquare(this.lShin + 2*r, 2*r, r);
			this.sThigh = RoundedSquare(this.lThigh + 2*r, 2*r, r);
			this.nsFoot = RoundedSquare(this.lFoot + 2*rr, 2*rr, rr);
			this.nsShin = RoundedSquare(this.lShin + 2*r, 2*r, r);
			this.nsThigh = RoundedSquare(this.lThigh + 2*r, 2*r, r);

			% Axes properties
			view(0, 89.9); % Fixes issue where OpenGL will sometimes not draw objects
			axis off;
		end % initialize

		function this = update(this, t)
		%UPDATE Update graphical objects.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Evaluate response at current time
			[x, u] = this.response.eval(t);

			% Parse states
			px = x(1);
			pz = x(2) + 0.015;
			q_sf = pi/2 + x(3);
			q_sr = this.lFoot + x(4);
			q_sa = q_sf + x(5);
			q_sk = q_sa + x(6);
			q_sh = q_sk + x(7);
			q_nsh = pi + q_sh - x(8);
			q_nsk = q_nsh - x(9);
			q_nsa = q_nsk - x(10);
			q_nsr = this.lFoot - x(11);

			% Update stance foot segment
			this.sFoot.reset;
			this.sFoot.translate(-this.sFoot.radius, -this.sFoot.height/2, 0);
			this.sFoot.scale(q_sr/this.lFoot, 1, 1);
			this.sFoot.rotate(0, 0, q_sf);
			this.sFoot.translate(px, pz, 0.01);
			this.sFoot.update;

			% Update stance shin segment
			this.sShin.reset;
			this.sShin.translate(-this.sShin.radius, -this.sShin.height/2, 0);
			this.sShin.rotate(0, 0, q_sa);
			this.sShin.translate(px + q_sr*cos(q_sf), pz + q_sr*sin(q_sf), 0.02);
			this.sShin.update;

			% Update stance thigh segment
			this.sThigh.reset;
			this.sThigh.translate(-this.sThigh.radius, -this.sThigh.height/2, 0);
			this.sThigh.rotate(0, 0, q_sk);
			this.sThigh.translate(px + q_sr*cos(q_sf) + this.lShin*cos(q_sa), pz + q_sr*sin(q_sf) + this.lShin*sin(q_sa), 0.03);
			this.sThigh.update;

			% Update torso
			this.torso.reset;
			this.torso.translate(-this.torso.radius, -this.torso.height/2, 0);
			this.torso.rotate(0, 0, q_sh);
			this.torso.translate(px + q_sr*cos(q_sf) + this.lShin*cos(q_sa) + this.lThigh*cos(q_sk), pz + q_sr*sin(q_sf) + this.lShin*sin(q_sa) + this.lThigh*sin(q_sk), 0.04);
			this.torso.update;

			% Update non-stance thigh segment
			this.nsThigh.reset;
			this.nsThigh.translate(-this.nsThigh.radius, -this.nsThigh.height/2, 0);
			this.nsThigh.rotate(0, 0, q_nsh);
			this.nsThigh.translate(px + q_sr*cos(q_sf) + this.lShin*cos(q_sa) + this.lThigh*cos(q_sk), pz + q_sr*sin(q_sf) + this.lShin*sin(q_sa) + this.lThigh*sin(q_sk), 0.07);
			this.nsThigh.update;

			% Update non-stance shin segment
			this.nsShin.reset;
			this.nsShin.translate(-this.nsShin.radius, -this.nsShin.height/2, 0);
			this.nsShin.rotate(0, 0, q_nsk);
			this.nsShin.translate(px + q_sr*cos(q_sf) + this.lShin*cos(q_sa) + this.lThigh*cos(q_sk) + this.lThigh*cos(q_nsh), pz + q_sr*sin(q_sf) + this.lShin*sin(q_sa) + this.lThigh*sin(q_sk) + this.lThigh*sin(q_nsh), 0.06);
			this.nsShin.update;

			% Update non-stance foot segment
			this.nsFoot.reset;
			this.nsFoot.translate(-this.nsFoot.radius, -this.nsFoot.height/2, 0);
			this.nsFoot.scale(q_nsr/this.lFoot, 1, 1);
			this.nsFoot.rotate(0, 0, q_nsa);
			this.nsFoot.translate(px + q_sr*cos(q_sf) + this.lShin*cos(q_sa) + this.lThigh*cos(q_sk) + this.lThigh*cos(q_nsh) + this.lShin*cos(q_nsk), pz + q_sr*sin(q_sf) + this.lShin*sin(q_sa) + this.lThigh*sin(q_sk) + this.lThigh*sin(q_nsh) + this.lShin*sin(q_nsk), 0.05);
			this.nsFoot.update;

			% Set axes limits
			w = 4;
			pe_com = pe_hip_vec(x);
			ylim(this.axes, ([-w, w] + w/2)*720/1280/2);
			xlim(this.axes, pe_com(1) + [-w, w]/2);
		end % update
	end % methods
end % classdef

%ATRIASSYSTEM ATRIAS bipedal robot system model.
%
% Copyright 2014 Mikhail S. Jones

classdef AtriasSystem < SecondOrderSystem

	% CONSTANT PROPERTIES ===================================================
	properties (Constant = true)
		tauLim = 100*0.0987*50 % Actuator torque limit
		DqLim = 7.88 % Actuator velocity limit
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function obj = AtriasSystem
		%ATRIASSYSTEM ATRIAS system model constructor.

			% Call superclass constructor
			obj = obj@SecondOrderSystem(...
				{'px', 'pz', 'qt', 'qLlA', 'qLlB', 'qLmA', 'qLmB', 'qRlA', 'qRlB', 'qRmA', 'qRmB'}, ...
				{'tauLmA', 'tauLmB', 'tauRmA', 'tauRmB'}, ...
				{'f', 'ss', 'ds'});

			% Set input bounds
			obj.inputLowerBounds = -obj.tauLim;
			obj.inputUpperBounds = obj.tauLim;
		end % AtriasSystem

		function [g, G, Gq] = constraintEquation(obj, x, xdot, m)
		%CONSTRAINTEQUATION ATRIAS domain specific constraint equations.

			switch m
			case 'ss'
				g = [[x(1)+cos(x(4))/2+cos(x(5))/2];
					[x(2)+sin(x(4))/2+sin(x(5))/2]];
				G = [[1,0,0,-sin(x(4))/2,-sin(x(5))/2,0,0,0,0,0,0];
					[0,1,0,cos(x(4))/2,cos(x(5))/2,0,0,0,0,0,0]];
				Gq = [[-(xdot(4)^2*cos(x(4)))/2-(xdot(5)^2*cos(x(5)))/2];
					[-(xdot(4)^2*sin(x(4)))/2-(xdot(5)^2*sin(x(5)))/2]];

			case 'ds'
				g = [[x(1)+cos(x(4))/2+cos(x(5))/2];
					[x(2)+sin(x(4))/2+sin(x(5))/2];
					[x(1)+cos(x(8))/2+cos(x(9))/2];
					[x(2)+sin(x(8))/2+sin(x(9))/2]];
				G = [[1,0,0,-sin(x(4))/2,-sin(x(5))/2,0,0,0,0,0,0];
					[0,1,0,cos(x(4))/2,cos(x(5))/2,0,0,0,0,0,0];
					[1,0,0,0,0,0,0,-sin(x(8))/2,-sin(x(9))/2,0,0];
					[0,1,0,0,0,0,0,cos(x(8))/2,cos(x(9))/2,0,0]];
				Gq = [[-(xdot(4)^2*cos(x(4)))/2-(xdot(5)^2*cos(x(5)))/2];
					[-(xdot(4)^2*sin(x(4)))/2-(xdot(5)^2*sin(x(5)))/2];
					[-(xdot(8)^2*cos(x(8)))/2-(xdot(9)^2*cos(x(9)))/2];
					[-(xdot(8)^2*sin(x(8)))/2-(xdot(9)^2*sin(x(9)))/2]];

			otherwise
				g = [];
				G = [];
				Gq = [];
			end % switch
		end % constraintEquation

		function [M, f] = secondOrderStateEquation(this, t, x, xdot, u, m)
		%SECONDORDERSTATEEQUATION The system model state equation.

			% States
			px = x(1);
			pz = x(2);
			qt = x(3);
			qLlA = x(4);
			qLlB = x(5);
			qLmA = x(6);
			qLmB = x(7);
			qRlA = x(8);
			qRlB = x(9);
			qRmA = x(10);
			qRmB = x(11);
			Dpx = xdot(1);
			Dpz = xdot(2);
			Dqt = xdot(3);
			DqLlA = xdot(4);
			DqLlB = xdot(5);
			DqLmA = xdot(6);
			DqLmB = xdot(7);
			DqRlA = xdot(8);
			DqRlB = xdot(9);
			DqRmA = xdot(10);
			DqRmB = xdot(11);

			% Inputs
			tauLmA = u(1);
			tauLmB = u(2);
			tauRmA = u(3);
			tauRmB = u(4);

			% Parameters
			ks = 4400;
			ba = 19;

			% Inertia matrix
			M(11,11) = 0*x(1);
			M(1,1) = 61.830483450000001670332494541071;
			M(1,3) = -7.4431987044658158371748090599311*sin(qt);
			M(1,4) = -0.36411484*sin(qLlA);
			M(1,5) = -0.4097491*sin(qLlB);
			M(1,8) = -0.36411484*sin(qRlA);
			M(1,9) = -0.4097491*sin(qRlB);
			M(2,2) = 61.830483450000001670332494541071;
			M(2,3) = 7.4431987044658158371748090599311*cos(qt);
			M(2,4) = 0.36411484*cos(qLlA);
			M(2,5) = 0.4097491*cos(qLlB);
			M(2,8) = 0.36411484*cos(qRlA);
			M(2,9) = 0.4097491*cos(qRlB);
			M(3,1) = -7.4431987044658158371748090599311*sin(qt);
			M(3,2) = 7.4431987044658158371748090599311*cos(qt);
			M(3,3) = 4.6204784300456799473667037530356;
			M(4,1) = -0.36411484*sin(qLlA);
			M(4,2) = 0.36411484*cos(qLlA);
			M(4,4) = 0.134267356416;
			M(4,5) = 0.05759972*cos(qLlA - qLlB);
			M(5,1) = -0.4097491*sin(qLlB);
			M(5,2) = 0.4097491*cos(qLlB);
			M(5,4) = 0.05759972*cos(qLlA - qLlB);
			M(5,5) = 0.17952863105;
			M(6,6) = 1.5392;
			M(7,7) = 1.5392;
			M(8,1) = -0.36411484*sin(qRlA);
			M(8,2) = 0.36411484*cos(qRlA);
			M(8,8) = 0.134267356416;
			M(8,9) = 0.05759972*cos(qRlA - qRlB);
			M(9,1) = -0.4097491*sin(qRlB);
			M(9,2) = 0.4097491*cos(qRlB);
			M(9,8) = 0.05759972*cos(qRlA - qRlB);
			M(9,9) = 0.17952863105;
			M(10,10) = 1.5392;
			M(11,11) = 1.5392;

			% Gravity and other terms
			f(11,1) = 0*x(1);
			f(1,1) = 0.36411484*DqLlA^2*cos(qLlA) + 0.4097491*DqLlB^2*cos(qLlB) + 0.36411484*DqRlA^2*cos(qRlA) + 0.4097491*DqRlB^2*cos(qRlB) + 7.4431987044658158371748090599311*Dqt^2*cos(qt);
			f(2,1) = 0.36411484*DqLlA^2*sin(qLlA) + 0.4097491*DqLlB^2*sin(qLlB) + 0.36411484*DqRlA^2*sin(qRlA) + 0.4097491*DqRlB^2*sin(qRlB) + 7.4431987044658158371748090599311*Dqt^2*sin(qt) - 606.55704264450003293518420832697;
			f(3,1) = DqLmA*ba - 1.0*tauLmB - 1.0*tauRmA - 1.0*tauRmB - 73.017779290809660371403357483504*cos(qt) - 1.0*tauLmA + DqLmB*ba + DqRmA*ba + DqRmB*ba - 4.0*Dqt*ba;
			f(4,1) = 1.49*DqLmA - 1.49*DqLlA - 3.5719665804000003817009201156907*cos(qLlA) - 0.05759972*DqLlB^2*sin(qLlA - qLlB) - 1.0*ks*qLlA + ks*qLmA;
			f(5,1) = 1.49*DqLmB - 1.49*DqLlB - 4.0196386710000001367681932151754*cos(qLlB) + 0.05759972*DqLlA^2*sin(qLlA - qLlB) - 1.0*ks*qLlB + ks*qLmB;
			f(6,1) = 1.49*DqLlA - 1.49*DqLmA + tauLmA - 0.5*ba*(2.0*DqLmA - 2.0*Dqt) + 0.5*ks*(2.0*qLlA - 2.0*qLmA);
			f(7,1) = 1.49*DqLlB - 1.49*DqLmB + tauLmB - 0.5*ba*(2.0*DqLmB - 2.0*Dqt) + 0.5*ks*(2.0*qLlB - 2.0*qLmB);
			f(8,1) = 1.49*DqRmA - 1.49*DqRlA - 3.5719665804000003817009201156907*cos(qRlA) - 0.05759972*DqRlB^2*sin(qRlA - qRlB) - 1.0*ks*qRlA + ks*qRmA;
			f(9,1) = 1.49*DqRmB - 1.49*DqRlB - 4.0196386710000001367681932151754*cos(qRlB) + 0.05759972*DqRlA^2*sin(qRlA - qRlB) - 1.0*ks*qRlB + ks*qRmB;
			f(10,1) = 1.49*DqRlA - 1.49*DqRmA + tauRmA - 0.5*ba*(2.0*DqRmA - 2.0*Dqt) + 0.5*ks*(2.0*qRlA - 2.0*qRmA);
			f(11,1) = 1.49*DqRlB - 1.49*DqRmB + tauRmB - 0.5*ba*(2.0*DqRmB - 2.0*Dqt) + 0.5*ks*(2.0*qRlB - 2.0*qRmB);
		end % secondOrderStateEquation
	end % methods
end % classdef

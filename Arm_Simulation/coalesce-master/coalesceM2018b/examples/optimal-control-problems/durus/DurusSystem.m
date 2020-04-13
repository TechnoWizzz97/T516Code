classdef DurusSystem < SecondOrderSystem
%DURUSSYSTEM Durus system model.
%
% Description:
%
%
% Copyright 2014 Mikhail S. Jones

	properties
		k_sp = 20000;
		b_sp = 200;
	end % properties

	methods
		function this = DurusSystem
		%DURUSSYSTEM Durus system model.
		%
		% Copyright 2014 Mikhail S. Jones

			% Define inputs and outputs
			x = {'p_x', 'p_z', 'q_sf' , 'q_sr' , 'q_sa', 'q_sk', 'q_sh', ...
				'q_nsh' , 'q_nsk' , 'q_nsa', 'q_nsr'};
			u = {'tau_1', 'tau_2', 'tau_3', 'tau_4', 'tau_5', 'tau_6'};
			m = {'ss', 'ds'};

			% Call superclass constructor
			this = this@SecondOrderSystem(x, u, m);

			% Set input bounds
			this.setInputBounds(-100*ones(6,1), 100*ones(6,1));
		end % DurusSystem

		function [g, G, Gq] = constraintEquation(this, x, m)

			switch m
			case 'ss'
				g = [h_sf_sca(x); h_nsr_sca(x)];
				G = [Jh_sf_mat(x); Jh_nsr_mat(x)];
				Gq = [dJh_sf_mat(x); dJh_nsr_mat(x)]*x(12:22);
			case 'ds'
				g = [h_sf_sca(x); h_nsf_sca(x)];
				G = [Jh_sf_mat(x); Jh_nsf_mat(x)];
				Gq = [dJh_sf_mat(x); dJh_nsf_mat(x)]*x(12:22);
			end % switch
		end % constraintEquation

		function [M, f] = secondOrderStateEquation(this, t, x, dx, u, m)
		%SECONDORDERSTATEEQUATION The system state equation.
		%
		% Copyright 2014 Mikhail S. Jones

			% Manipulator equation matrices
			M = De_mat(x);
			C = Ce_mat([x; dx]);
			G = Ge_vec(x);

			% Springs torques
			B_sp(1:11,1) = x(1)*0;
			B_sp(4,1) = -this.k_sp*x(4) - this.b_sp*dx(4);
			B_sp(11,1) = -this.k_sp*x(11) - this.b_sp*dx(11);
			% B_sp(5:10) = -0.75*dx(5:10);

			% Motor torques
			B(1:11,1) = x(1)*0;
			B(5:10) = u;

			% Combine terms
			f = B_sp + B - C*dx - G;
		end % secondOrderStateEquation
	end % methods
end % classdef

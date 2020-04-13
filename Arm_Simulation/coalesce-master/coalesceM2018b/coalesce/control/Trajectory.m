%TRAJECTORY
%
% Copyright 2014 Mikhail S. Jones

classdef Trajectory

	% PROTECTED PROPERTIES ==================================================
	properties
		pp
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function this = Trajectory(t, x)
		%TRAJECTORY Trajetory constructor.

			this.pp = spline(t, x);
		end % Trajectory

		function x = ppval(this, t)
		%PPVAL

			% Bound dependent to avoid extrapolation
			t = min(max(t, min(this.pp.breaks)), max(this.pp.breaks));

			x = ppval(this.pp, t);
		end % ppval

		function x = eval(this, t)
		%EVAL

			% Bound dependent to avoid extrapolation
			t = min(max(t, min(this.pp.breaks)), max(this.pp.breaks));
			t = reshape(t, [], 1);

			br = this.pp.breaks.';
			cf = this.pp.coefs;
			[~, inds] = histc(t, [-inf; br(2:end-1); inf]);
	    t_shf = t.' - br(inds);
	    zero  = ones(size(t_shf));
	    one   = t_shf;
	    two   = one.*t_shf;
	    three = two.*t_shf;

	    x = sum([three two one zero].*cf(inds,:), 2);
		end % eval

		function this = fnder(this)
		%FNDER

			% Differentiate trajectory
			this.pp = fnder(this.pp);
		end % fnder

		function plot(this)
		%PLOT

			% Plot trajectory
			breaks = linspace(this.pp.breaks(1), this.pp.breaks(end), 100*this.pp.pieces);
			plot(breaks, ppval(this, breaks));
		end % plot

		function this = horzcat(this, varargin)
		%HORZCAT

			for i = 1:numel(varargin)
				% Check breaks match
				% TODO

				this.pp.breaks = [this.pp.breaks, varargin{i}.pp.breaks(2:end)];
				this.pp.coefs = [this.pp.coefs; varargin{i}.pp.coefs];
				this.pp.pieces = this.pp.pieces + varargin{i}.pp.pieces;
			end % if
		end % horzcat
	end % methods
end % classdef

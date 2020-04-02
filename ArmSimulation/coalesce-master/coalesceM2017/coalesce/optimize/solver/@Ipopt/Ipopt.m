classdef Ipopt < Solver
%IPOPT Provides an interface for COALESCE objects.
%
% Copyright 2013-2014 Mikhail S. Jones

	% PUBLIC PROPERTIES =====================================================
	properties
	end % properties

	% PUBLIC METHODS ========================================================
	methods
		function this = Ipopt(nlp)
		%IPOPT Creates a optimization solver object.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Solver(nlp);

			% Set default IPOPT options
			this.options = struct;
			this.options.ipopt.hessian_approximation = 'limited-memory';
			this.options.ipopt.limited_memory_max_history = 6;
		  this.options.ipopt.limited_memory_max_skipping = 6;
		  this.options.ipopt.max_iter = 1e3;
		  this.options.ipopt.ma57_pre_alloc = 2;

		  % Note: mumps linear solver is deterministic but approximately twice
  		% as slow as ma57 in many cases. ma57 is non-deterministic due to
  		%	parallelization and memory allocation variations.
			this.options.ipopt.linear_solver = 'ma57';

			% Note: Can use first or second-order check, useful for debugging.
			this.options.ipopt.derivative_test = 'none';
		end % Ipopt

		function setOptions(this, varargin)
		%SETOPTIONS Set optimizer options.

			% Set IPOPT options
			for i = 1:2:numel(nargin)
				this.options.ipopt.(varargin{i}) = varargin{i+1};
			end % for
		end % setOptions
	end % methods
end % classdef

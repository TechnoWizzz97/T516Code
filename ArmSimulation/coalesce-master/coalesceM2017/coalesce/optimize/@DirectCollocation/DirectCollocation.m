classdef DirectCollocation < Nlp
%DIRECTCOLLOCATION Defines a direct collocation optimization problem.
%
% Copyright 2013-2014 Mikhail S. Jones

	properties (SetAccess = public, GetAccess = public)
		nNodes@double scalar % Number of collocation nodes
		inputVariable@Variable % References to input variable objects
		stateVariable@Variable % References to state variable objects
		timeVariable@Variable % Reference to time variable objects
	end % properties

	methods (Access = public)
		function this = DirectCollocation(nNodes, varargin)
		%DIRECTCOLLOCATION Direct Collocation optimization problem constructor.
		%
		% Syntax:
		%   obj = DirectCollocation(nNodes)
		%
		% Required Input Arguments:
		%   nNode - (DOUBLE) Number of nodes in phase
		%
		% Optional Input Arguments:
		%   name - (CHAR) Trajectory optimization problem description or name.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Call superclass constructor
			this = this@Nlp(varargin{:});

			% Sub class properties
			this.nNodes = nNodes;
		end % DirectCollocation
	end % methods
end % classdef

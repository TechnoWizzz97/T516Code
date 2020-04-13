%SOLVER Provides a solver interface for COALESCE Nlp objects.
%
% Description:
%   Abstract class defining solver interfaces to the COALESCE NLPs.
%
% Copyright 2013-2014 Mikhail S. Jones

classdef (Abstract = true) Solver < handle

	% PUBLIC PROPERTIES =====================================================
	properties
		nlp@Nlp scalar
		options@struct
		info
	end % properties

	% ABSTRACT METHODS ======================================================
	methods (Abstract = true)
		export
		solve
	end % methods

	% PUBLIC METHODS ========================================================
	methods
		function this = Solver(nlp)
		%SOLVER Creates a optimization solver object.
		%
		% Syntax:
		%   obj = Solver(nlp)
		%
		% Input Arguments:
		%   nlp - (NLP) A coalesce problem object
		%
		% Description:
		%   Construct a solver object used for optimizing nlp objects.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% Set object properties
			this.nlp = nlp;

			% Create directory for auto generated files if it doesn't exist
			if ~exist('auto', 'dir')
				% Make directory
				mkdir('auto');
			end % if

			% Set path to auto generated functions
			addpath('auto/');

			% User feedback
			fprintf('Solver object constructed!\n');
		end % Solver

		function delete(~)
		%DELETE Delete the object.
		%
		% Description:
		%   DELETE does not need to called directly, as it is called when
		%   the object is cleared.
		%
		% Copyright 2013-2014 Mikhail S. Jones

			% User feedback
			fprintf('Solver object deleted!\n');
		end % delete
	end % methods
end % classdef

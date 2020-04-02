function feasibility(this, tol)
%FEASIBILITY Display constraint feasibility summary.
%
% Syntax:
%   obj.feasibility(tol)
%
% Input Arguments:
%   tol - (DOUBLE) Feasibility tolerance cutoff
%
% Description:
%   Displays all constraints that do not meet the specified
%   tolerance with id, name, and values compared to defined bounds.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Check number of input arguments
	if nargin == 1
		% Default feasibility tolerance
		tol = 1e-6;
	end % if

	% Construct divider line strings
	majorDivider = [repmat('=', 1, 75), '\n'];
	minorDivider = [repmat('-', 1, 75), '\n'];

	% Display feasibility summary header
	fprintf('\n');
	fprintf(majorDivider);
	fprintf([repmat(' ',1,28) '[\bFEASIBILITY SUMMARY]\b \n']);
	fprintf(majorDivider);
	fprintf('* Tolerance: [\b%0.1e]\b \n\n', tol);

	% Loop through all constraint sets
	for iCon = 1:numel(this.nlp.constraint)
		% Display constraint group sub-header
		fprintf('%s\n', upper(this.nlp.constraint(iCon).description));
		fprintf(minorDivider);

		% Reset and reallocate bound arrays
		lb = this.nlp.constraint(iCon).lowerBound';
		ub = this.nlp.constraint(iCon).upperBound';

		% Evaluate constraint expression
		c = eval(this.nlp.constraint(iCon));

		% Find infeasible constraints
		isFeasible = logical(((lb - tol) <= c) & (c <= (ub + tol)));

		% Check if all constraints are feasible
		if all(isFeasible)
			% Display all feasible notification
			fprintf('\tAll constraints feasible to within tolerance.\n');

		else
			% Display constraints within set
			for i = 1:this.nlp.constraint(iCon).expression.length
				if ~isFeasible(i)
					fprintf('%-12.12s %12.12s <= [\b%.4e]\b <= %s \n', ...
						['  ' num2str(i) ')'], num2str(lb(i)), c(i), num2str(ub(i)));
				end % if
			end % for
		end % if

		fprintf('\n');
	end % for

	% Display feasibility summary footer
	fprintf(majorDivider);
	fprintf('\n');
end % feasibility

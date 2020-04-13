function display(this)
%DISPLAY Display design variable solution solution.
%
% Syntax:
%		obj
%   obj.display
%
% Description:
%   Displays all design variable solutions along with name and id.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Construct divider line strings
	majorDivider = [repmat('=', 1, 75), '\n'];
	minorDivider = [repmat('-', 1, 75), '\n'];

	% Display solution summary header
	fprintf('\n');
	fprintf(majorDivider);
	fprintf([repmat(' ', 1, 30) '[\bSOLVER SUMMARY]\b \n']);
	fprintf(majorDivider);
	fprintf('* Solver: [\b%s]\b \n\n', class(this));

	% Display solution summary
	for i = 1:numel(this.nlp.variable)
		fprintf('[\b%s]\b \n', upper(this.nlp.variable(i).description));
		fprintf(minorDivider);

		% Display variables within set
		for j = 1:this.nlp.variable(i).length
			fprintf('%-12.12s %12.12s = [\b%.4e]\b \n', ...
				['  ' num2str(j) ')'], ...
				char(this.nlp.variable(i)), ... % TODO
				this.nlp.variable(i).solution(j));
		end % for
		fprintf('\n');
	end % for

	% Display solution summary footer
	fprintf(majorDivider);
	fprintf('\n');
end % display

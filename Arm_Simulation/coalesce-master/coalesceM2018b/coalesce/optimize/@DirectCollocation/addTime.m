function expression = addTime(this)
%ADDTIME Add time to trajectory optimization problem.
%
% Description:
%   Wrapper for addVariable method that also adds variable to time list.
%
% Copyright 2013-2014 Mikhail S. Jones

	expression = this.addVariable(1, 0, Inf, ...
		'Description', sprintf('Phase %d Duration', numel(this.timeVariable)+1));

	% Add time object to phase object
	this.timeVariable(end+1) = expression;
end % addTime

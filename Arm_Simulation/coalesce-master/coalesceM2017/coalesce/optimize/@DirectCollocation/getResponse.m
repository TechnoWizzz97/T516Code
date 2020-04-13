function response = getResponse(this)
%GETRESPONSE Export direct collocation solution as response object.
%
% Copyright 2013-2014 Mikhail S. Jones

	% Loop through phases
	for iTime = 1:numel(this.timeVariable)
		% Determine time interval
		if iTime == 1
			tStar{iTime} = linspace(0, eval(this.timeVariable(iTime)), this.nNodes);
		else
			tStar{iTime} = linspace(eval(this.timeVariable(iTime-1)), eval(this.timeVariable(iTime)), this.nNodes);
		end % if

		% States
		xStar{iTime} = reshape(eval(this.stateVariable(:,iTime)), [], this.nNodes);

		% Inputs
		uStar{iTime} = reshape(eval(this.inputVariable(:,iTime)), [], this.nNodes);
	end % for

	% Construct response object
	response = Response(tStar, xStar, uStar, {this.options.name}, ...
		{this.stateVariable.description}, ...
		{this.inputVariable.description});
end % getResponse

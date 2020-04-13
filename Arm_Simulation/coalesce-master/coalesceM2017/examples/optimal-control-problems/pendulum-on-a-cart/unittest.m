function unittest
%UNITTEST Run unit test.
%
% Copyright 2014 Mikhail S. Jones

	% Run tests
	clear all; runPassive(10);
	clear all; runFixedPointTiLqr(3);
	clear all; runSwingUpTvLqr(3);
	clear all; runSideStepTvLqr(3);
	clear all; runSwingTransverseLqr(3);

end % unittest

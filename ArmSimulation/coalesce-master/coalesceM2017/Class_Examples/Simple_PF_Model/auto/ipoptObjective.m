function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (16-Jan-2019 21:05:08)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = (var(1).*sum((((var(614:1:663).^2.0)+(var(615:1:664).^2.0))./2.0)));
end % ipoptObjective
function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (10-Jan-2019 16:50:02)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = (var(1).*sum(((var(2:1:200)+var(3:1:201))./2.0)));
end % ipoptObjective
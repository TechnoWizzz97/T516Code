function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (18-Jan-2019 14:05:32)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = (var(1).*sum(((var(204:1:303)+var(205:1:304))./2.0)));
end % ipoptObjective
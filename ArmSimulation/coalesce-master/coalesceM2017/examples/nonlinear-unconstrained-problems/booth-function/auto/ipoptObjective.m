function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:27)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = ((((var(1)+(2.0.*var(2)))-7.0).^2.0)+((((2.0.*var(1))+var(2))-5.0).^2.0));
end % ipoptObjective
function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:23)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = ((((((var(2)-(0.12918450914398066.*(var(1).^2.0)))+(1.5915494309189535.*var(1)))-6.0).^2.0)+(9.602112642270262.*cos(var(1))))+10.0);
end % ipoptObjective
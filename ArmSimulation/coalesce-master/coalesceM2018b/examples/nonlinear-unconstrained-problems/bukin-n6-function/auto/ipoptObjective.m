function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (10-Jan-2019 16:49:37)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = ((100.0.*sqrt(abs((var(2)-(0.01.*(var(1).^2.0))))))+(0.01.*abs((var(1)+10.0))));
end % ipoptObjective
function [f] = ipoptObjective(var)
%IPOPTOBJECTIVE
%
% Auto-generated by COALESCE package (10-Jan-2019 17:07:06)
%
% Copyright 2013-2014 Mikhail S. Jones

	f(1,1) = 0; % Pre-allocation
	f(1) = ((var(1).*sum(((((((var(2:1:200).^2.0)+(var(202:1:400).^2.0))+(var(402:1:600).^2.0))+(var(602:1:800).^2.0))+((((var(3:1:201).^2.0)+(var(203:1:401).^2.0))+(var(403:1:601).^2.0))+(var(603:1:801).^2.0)))./2.0)))./(588.6.*var(7802)));
end % ipoptObjective
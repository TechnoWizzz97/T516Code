function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (10-Jan-2019 16:50:10)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(802,1) = 0; % Pre-allocation
	c(1:200) = (var(602:1:801)-((((1.0-(var(202:1:401).^2.0)).*var(402:1:601))-var(202:1:401))+var(2:1:201)));
	c(201:400) = (var(802:1:1001)-((((1.0-(var(402:1:601).^2.0)).*var(402:1:601))-var(402:1:601))+var(2:1:201)));
	c(401:599) = ((var(203:1:401)-var(202:1:400))-(((var(1)./199.0).*(var(402:1:600)+var(403:1:601)))./2.0));
	c(600:798) = ((var(403:1:601)-var(402:1:600))-(((var(1)./199.0).*(var(602:1:800)+var(603:1:801)))./2.0));
	c(799) = var(202);
	c(800) = var(402);
	c(801) = var(401);
	c(802) = var(601);
end % ipoptConstraints
function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (09-Jan-2019 14:19:17)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(1606,1) = 0; % Pre-allocation
	c(1:199) = ((var(403:1:601)-var(402:1:600))-(((var(1)./199.0).*(var(1002:1:1200)+var(1003:1:1201)))./2.0));
	c(200:398) = ((var(603:1:801)-var(602:1:800))-(((var(1)./199.0).*(var(1202:1:1400)+var(1203:1:1401)))./2.0));
	c(399:597) = ((var(803:1:1001)-var(802:1:1000))-(((var(1)./199.0).*(var(1402:1:1600)+var(1403:1:1601)))./2.0));
	c(598:796) = ((var(1003:1:1201)-var(1002:1:1200))-(((var(1)./199.0).*((-((var(2:1:200)+var(202:1:400))).*sin(var(802:1:1000)))+(-((var(3:1:201)+var(203:1:401))).*sin(var(803:1:1001)))))./2.0));
	c(797:995) = ((var(1203:1:1401)-var(1202:1:1400))-(((var(1)./199.0).*((-9.81+((var(2:1:200)+var(202:1:400)).*cos(var(802:1:1000))))+(-9.81+((var(3:1:201)+var(203:1:401)).*cos(var(803:1:1001))))))./2.0));
	c(996:1194) = ((var(1403:1:1601)-var(1402:1:1600))-(((var(1)./199.0).*((((var(202:1:400)-var(2:1:200)).*0.25)./0.10000000000000001)+(((var(203:1:401)-var(3:1:201)).*0.25)./0.10000000000000001)))./2.0));
	c(1195:1394) = (var(602:1:801)-(0.125.*sin(var(802:1:1001))));
	c(1395:1594) = (var(602:1:801)+(0.125.*sin(var(802:1:1001))));
	c(1595) = var(402);
	c(1596) = var(602);
	c(1597) = var(802);
	c(1598) = var(1002);
	c(1599) = var(1202);
	c(1600) = var(1402);
	c(1601) = var(601);
	c(1602) = var(801);
	c(1603) = var(1001);
	c(1604) = var(1201);
	c(1605) = var(1401);
	c(1606) = var(1601);
end % ipoptConstraints
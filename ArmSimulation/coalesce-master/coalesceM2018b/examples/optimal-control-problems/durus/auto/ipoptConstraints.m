function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (10-Jan-2019 15:47:28)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(2408,1) = 0; % Pre-allocation
	c(1:200) = (((((2.0.*var(1802:1:2001))+((-((5.0.*sin(var(402:1:601))))./18.0).*var(2002:1:2201)))+((-(sin(var(602:1:801)))./6.0).*var(2202:1:2401)))+((-(sin(var(802:1:1001)))./18.0).*var(2402:1:2601)))-((((var(2:1:201)-(var(1002:1:1201)./10.0))+(((5.0.*(var(1202:1:1401).^2.0)).*cos(var(402:1:601)))./18.0))+(((var(1402:1:1601).^2.0).*cos(var(602:1:801)))./6.0))+(((var(1602:1:1801).^2.0).*cos(var(802:1:1001)))./18.0)));
	c(201:400) = ((((((-((5.0.*sin(var(402:1:601))))./18.0).*var(1802:1:2001))+(0.08641975308641975.*var(2002:1:2201)))+((cos((var(402:1:601)-var(602:1:801)))./18.0).*var(2202:1:2401)))+((cos((var(402:1:601)-var(802:1:1001)))./54.0).*var(2402:1:2601)))-(((((var(1402:1:1601)./300.0)-(var(1202:1:1401)./150.0))-((109.0.*cos(var(402:1:601)))./40.0))-(((var(1402:1:1601).^2.0).*sin((var(402:1:601)-var(602:1:801))))./18.0))-(((var(1602:1:1801).^2.0).*sin((var(402:1:601)-var(802:1:1001))))./54.0)));
	c(401:600) = ((((((-(sin(var(602:1:801)))./6.0).*var(1802:1:2001))+((cos((var(402:1:601)-var(602:1:801)))./18.0).*var(2002:1:2201)))+(0.049382716049382713.*var(2202:1:2401)))+((cos((var(602:1:801)-var(802:1:1001)))./54.0).*var(2402:1:2601)))-((((((var(1202:1:1401)./300.0)-(var(1402:1:1601)./150.0))+(var(1602:1:1801)./300.0))-((327.0.*cos(var(602:1:801)))./200.0))+(((var(1202:1:1401).^2.0).*sin((var(402:1:601)-var(602:1:801))))./18.0))-(((var(1602:1:1801).^2.0).*sin((var(602:1:801)-var(802:1:1001))))./54.0)));
	c(601:800) = ((((((-(sin(var(802:1:1001)))./18.0).*var(1802:1:2001))+((cos((var(402:1:601)-var(802:1:1001)))./54.0).*var(2002:1:2201)))+((cos((var(602:1:801)-var(802:1:1001)))./54.0).*var(2202:1:2401)))+(0.012345679012345678.*var(2402:1:2601)))-(((((var(1402:1:1601)./300.0)-(var(1602:1:1801)./300.0))-((109.0.*cos(var(802:1:1001)))./200.0))+(((var(1202:1:1401).^2.0).*sin((var(402:1:601)-var(802:1:1001))))./54.0))+(((var(1402:1:1601).^2.0).*sin((var(602:1:801)-var(802:1:1001))))./54.0)));
	c(801:999) = ((var(203:1:401)-var(202:1:400))-(((var(1)./199.0).*(var(1002:1:1200)+var(1003:1:1201)))./2.0));
	c(1000:1198) = ((var(1003:1:1201)-var(1002:1:1200))-(((var(1)./199.0).*(var(1802:1:2000)+var(1803:1:2001)))./2.0));
	c(1199:1397) = ((var(403:1:601)-var(402:1:600))-(((var(1)./199.0).*(var(1202:1:1400)+var(1203:1:1401)))./2.0));
	c(1398:1596) = ((var(1203:1:1401)-var(1202:1:1400))-(((var(1)./199.0).*(var(2002:1:2200)+var(2003:1:2201)))./2.0));
	c(1597:1795) = ((var(603:1:801)-var(602:1:800))-(((var(1)./199.0).*(var(1402:1:1600)+var(1403:1:1601)))./2.0));
	c(1796:1994) = ((var(1403:1:1601)-var(1402:1:1600))-(((var(1)./199.0).*(var(2202:1:2400)+var(2203:1:2401)))./2.0));
	c(1995:2193) = ((var(803:1:1001)-var(802:1:1000))-(((var(1)./199.0).*(var(1602:1:1800)+var(1603:1:1801)))./2.0));
	c(2194:2392) = ((var(1603:1:1801)-var(1602:1:1800))-(((var(1)./199.0).*(var(2402:1:2600)+var(2403:1:2601)))./2.0));
	c(2393) = var(202);
	c(2394) = var(402);
	c(2395) = var(602);
	c(2396) = var(802);
	c(2397) = var(1002);
	c(2398) = var(1202);
	c(2399) = var(1402);
	c(2400) = var(1602);
	c(2401) = var(401);
	c(2402) = var(601);
	c(2403) = var(801);
	c(2404) = var(1001);
	c(2405) = var(1201);
	c(2406) = var(1401);
	c(2407) = var(1601);
	c(2408) = var(1801);
end % ipoptConstraints
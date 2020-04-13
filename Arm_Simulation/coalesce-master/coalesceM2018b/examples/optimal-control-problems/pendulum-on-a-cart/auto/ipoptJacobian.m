function [J] = ipoptJacobian(var)
%IPOPTJACOBIAN
%
% Auto-generated by COALESCE package (10-Jan-2019 16:50:59)
%
% Copyright 2013-2014 Mikhail S. Jones

	iJ = [1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,1:1:200,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,201:1:400,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,401:1:600,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,601:1:800,801:1:999,801:1:999,801:1:999,801:1:999,801:1:999,1000:1:1198,1000:1:1198,1000:1:1198,1000:1:1198,1000:1:1198,1199:1:1397,1199:1:1397,1199:1:1397,1199:1:1397,1199:1:1397,1398:1:1596,1398:1:1596,1398:1:1596,1398:1:1596,1398:1:1596,1597:1:1795,1597:1:1795,1597:1:1795,1597:1:1795,1597:1:1795,1796:1:1994,1796:1:1994,1796:1:1994,1796:1:1994,1796:1:1994,1995:1:2193,1995:1:2193,1995:1:2193,1995:1:2193,1995:1:2193,2194:1:2392,2194:1:2392,2194:1:2392,2194:1:2392,2194:1:2392,2393,2394,2395,2396,2397,2398,2399,2400,2401,2402,2403,2404,2405,2406,2407,2408]';
	jJ = [1802:1:2001,402:1:601,2002:1:2201,602:1:801,2202:1:2401,802:1:1001,2402:1:2601,2:1:201,1002:1:1201,1202:1:1401,1402:1:1601,1602:1:1801,402:1:601,1802:1:2001,2002:1:2201,602:1:801,2202:1:2401,802:1:1001,2402:1:2601,1402:1:1601,1202:1:1401,1602:1:1801,602:1:801,1802:1:2001,402:1:601,2002:1:2201,2202:1:2401,802:1:1001,2402:1:2601,1202:1:1401,1402:1:1601,1602:1:1801,802:1:1001,1802:1:2001,402:1:601,2002:1:2201,602:1:801,2202:1:2401,2402:1:2601,1402:1:1601,1602:1:1801,1202:1:1401,203:1:401,202:1:400,1+zeros(1,199),1002:1:1200,1003:1:1201,1003:1:1201,1002:1:1200,1+zeros(1,199),1802:1:2000,1803:1:2001,403:1:601,402:1:600,1+zeros(1,199),1202:1:1400,1203:1:1401,1203:1:1401,1202:1:1400,1+zeros(1,199),2002:1:2200,2003:1:2201,603:1:801,602:1:800,1+zeros(1,199),1402:1:1600,1403:1:1601,1403:1:1601,1402:1:1600,1+zeros(1,199),2202:1:2400,2203:1:2401,803:1:1001,802:1:1000,1+zeros(1,199),1602:1:1800,1603:1:1801,1603:1:1801,1602:1:1800,1+zeros(1,199),2402:1:2600,2403:1:2601,202,402,602,802,1002,1202,1402,1602,401,601,801,1001,1201,1401,1601,1801]';
	sJ(16376,1) = 0; % Pre-allocation
	sJ(1:200) = 2.0;
	sJ(201:400) = ((var(2002:1:2201).*(-((5.0.*cos(var(402:1:601))))./18.0))-(((5.0.*(var(1202:1:1401).^2.0)).*-(sin(var(402:1:601))))./18.0));
	sJ(401:600) = (-((5.0.*sin(var(402:1:601))))./18.0);
	sJ(601:800) = ((var(2202:1:2401).*(-(cos(var(602:1:801)))./6.0))-(((var(1402:1:1601).^2.0).*-(sin(var(602:1:801))))./6.0));
	sJ(801:1000) = (-(sin(var(602:1:801)))./6.0);
	sJ(1001:1200) = ((var(2402:1:2601).*(-(cos(var(802:1:1001)))./18.0))-(((var(1602:1:1801).^2.0).*-(sin(var(802:1:1001))))./18.0));
	sJ(1201:1400) = (-(sin(var(802:1:1001)))./18.0);
	sJ(1401:1600) = -1.0;
	sJ(1601:1800) = 0.10000000000000001;
	sJ(1801:2000) = -(((cos(var(402:1:601)).*(5.0.*(2.0.*var(1202:1:1401))))./18.0));
	sJ(2001:2200) = -(((cos(var(602:1:801)).*(2.0.*var(1402:1:1601)))./6.0));
	sJ(2201:2400) = -(((cos(var(802:1:1001)).*(2.0.*var(1602:1:1801)))./18.0));
	sJ(2401:2600) = ((((var(1802:1:2001).*(-((5.0.*cos(var(402:1:601))))./18.0))+(var(2202:1:2401).*(-(sin((var(402:1:601)-var(602:1:801))))./18.0)))+(var(2402:1:2601).*(-(sin((var(402:1:601)-var(802:1:1001))))./54.0)))-((-(((109.0.*-(sin(var(402:1:601))))./40.0))-(((var(1402:1:1601).^2.0).*cos((var(402:1:601)-var(602:1:801))))./18.0))-(((var(1602:1:1801).^2.0).*cos((var(402:1:601)-var(802:1:1001))))./54.0)));
	sJ(2601:2800) = (-((5.0.*sin(var(402:1:601))))./18.0);
	sJ(2801:3000) = 0.08641975308641975;
	sJ(3001:3200) = ((var(2202:1:2401).*(-(-(sin((var(402:1:601)-var(602:1:801)))))./18.0))--((((var(1402:1:1601).^2.0).*-(cos((var(402:1:601)-var(602:1:801)))))./18.0)));
	sJ(3201:3400) = (cos((var(402:1:601)-var(602:1:801)))./18.0);
	sJ(3401:3600) = ((var(2402:1:2601).*(-(-(sin((var(402:1:601)-var(802:1:1001)))))./54.0))--((((var(1602:1:1801).^2.0).*-(cos((var(402:1:601)-var(802:1:1001)))))./54.0)));
	sJ(3601:3800) = (cos((var(402:1:601)-var(802:1:1001)))./54.0);
	sJ(3801:4000) = -((0.0033333333333333335-((sin((var(402:1:601)-var(602:1:801))).*(2.0.*var(1402:1:1601)))./18.0)));
	sJ(4001:4200) = 0.0066666666666666671;
	sJ(4201:4400) = -(-(((sin((var(402:1:601)-var(802:1:1001))).*(2.0.*var(1602:1:1801)))./54.0)));
	sJ(4401:4600) = ((((var(1802:1:2001).*(-(cos(var(602:1:801)))./6.0))+(var(2002:1:2201).*(-(-(sin((var(402:1:601)-var(602:1:801)))))./18.0)))+(var(2402:1:2601).*(-(sin((var(602:1:801)-var(802:1:1001))))./54.0)))-((-(((327.0.*-(sin(var(602:1:801))))./200.0))+(((var(1202:1:1401).^2.0).*-(cos((var(402:1:601)-var(602:1:801)))))./18.0))-(((var(1602:1:1801).^2.0).*cos((var(602:1:801)-var(802:1:1001))))./54.0)));
	sJ(4601:4800) = (-(sin(var(602:1:801)))./6.0);
	sJ(4801:5000) = ((var(2002:1:2201).*(-(sin((var(402:1:601)-var(602:1:801))))./18.0))-(((var(1202:1:1401).^2.0).*cos((var(402:1:601)-var(602:1:801))))./18.0));
	sJ(5001:5200) = (cos((var(402:1:601)-var(602:1:801)))./18.0);
	sJ(5201:5400) = 0.049382716049382713;
	sJ(5401:5600) = ((var(2402:1:2601).*(-(-(sin((var(602:1:801)-var(802:1:1001)))))./54.0))--((((var(1602:1:1801).^2.0).*-(cos((var(602:1:801)-var(802:1:1001)))))./54.0)));
	sJ(5601:5800) = (cos((var(602:1:801)-var(802:1:1001)))./54.0);
	sJ(5801:6000) = -((0.0033333333333333335+((sin((var(402:1:601)-var(602:1:801))).*(2.0.*var(1202:1:1401)))./18.0)));
	sJ(6001:6200) = 0.0066666666666666671;
	sJ(6201:6400) = -((0.0033333333333333335-((sin((var(602:1:801)-var(802:1:1001))).*(2.0.*var(1602:1:1801)))./54.0)));
	sJ(6401:6600) = ((((var(1802:1:2001).*(-(cos(var(802:1:1001)))./18.0))+(var(2002:1:2201).*(-(-(sin((var(402:1:601)-var(802:1:1001)))))./54.0)))+(var(2202:1:2401).*(-(-(sin((var(602:1:801)-var(802:1:1001)))))./54.0)))-((-(((109.0.*-(sin(var(802:1:1001))))./200.0))+(((var(1202:1:1401).^2.0).*-(cos((var(402:1:601)-var(802:1:1001)))))./54.0))+(((var(1402:1:1601).^2.0).*-(cos((var(602:1:801)-var(802:1:1001)))))./54.0)));
	sJ(6601:6800) = (-(sin(var(802:1:1001)))./18.0);
	sJ(6801:7000) = ((var(2002:1:2201).*(-(sin((var(402:1:601)-var(802:1:1001))))./54.0))-(((var(1202:1:1401).^2.0).*cos((var(402:1:601)-var(802:1:1001))))./54.0));
	sJ(7001:7200) = (cos((var(402:1:601)-var(802:1:1001)))./54.0);
	sJ(7201:7400) = ((var(2202:1:2401).*(-(sin((var(602:1:801)-var(802:1:1001))))./54.0))-(((var(1402:1:1601).^2.0).*cos((var(602:1:801)-var(802:1:1001))))./54.0));
	sJ(7401:7600) = (cos((var(602:1:801)-var(802:1:1001)))./54.0);
	sJ(7601:7800) = 0.012345679012345678;
	sJ(7801:8000) = -((0.0033333333333333335+((sin((var(602:1:801)-var(802:1:1001))).*(2.0.*var(1402:1:1601)))./54.0)));
	sJ(8001:8200) = 0.0033333333333333335;
	sJ(8201:8400) = -(((sin((var(402:1:601)-var(802:1:1001))).*(2.0.*var(1202:1:1401)))./54.0));
	sJ(8401:8599) = 1.0;
	sJ(8600:8798) = -1.0;
	sJ(8799:8997) = -((((var(1002:1:1200)+var(1003:1:1201)).*0.0050251256281407036)./2.0));
	sJ(8998:9196) = -(((var(1)./199.0)./2.0));
	sJ(9197:9395) = -(((var(1)./199.0)./2.0));
	sJ(9396:9594) = 1.0;
	sJ(9595:9793) = -1.0;
	sJ(9794:9992) = -((((var(1802:1:2000)+var(1803:1:2001)).*0.0050251256281407036)./2.0));
	sJ(9993:10191) = -(((var(1)./199.0)./2.0));
	sJ(10192:10390) = -(((var(1)./199.0)./2.0));
	sJ(10391:10589) = 1.0;
	sJ(10590:10788) = -1.0;
	sJ(10789:10987) = -((((var(1202:1:1400)+var(1203:1:1401)).*0.0050251256281407036)./2.0));
	sJ(10988:11186) = -(((var(1)./199.0)./2.0));
	sJ(11187:11385) = -(((var(1)./199.0)./2.0));
	sJ(11386:11584) = 1.0;
	sJ(11585:11783) = -1.0;
	sJ(11784:11982) = -((((var(2002:1:2200)+var(2003:1:2201)).*0.0050251256281407036)./2.0));
	sJ(11983:12181) = -(((var(1)./199.0)./2.0));
	sJ(12182:12380) = -(((var(1)./199.0)./2.0));
	sJ(12381:12579) = 1.0;
	sJ(12580:12778) = -1.0;
	sJ(12779:12977) = -((((var(1402:1:1600)+var(1403:1:1601)).*0.0050251256281407036)./2.0));
	sJ(12978:13176) = -(((var(1)./199.0)./2.0));
	sJ(13177:13375) = -(((var(1)./199.0)./2.0));
	sJ(13376:13574) = 1.0;
	sJ(13575:13773) = -1.0;
	sJ(13774:13972) = -((((var(2202:1:2400)+var(2203:1:2401)).*0.0050251256281407036)./2.0));
	sJ(13973:14171) = -(((var(1)./199.0)./2.0));
	sJ(14172:14370) = -(((var(1)./199.0)./2.0));
	sJ(14371:14569) = 1.0;
	sJ(14570:14768) = -1.0;
	sJ(14769:14967) = -((((var(1602:1:1800)+var(1603:1:1801)).*0.0050251256281407036)./2.0));
	sJ(14968:15166) = -(((var(1)./199.0)./2.0));
	sJ(15167:15365) = -(((var(1)./199.0)./2.0));
	sJ(15366:15564) = 1.0;
	sJ(15565:15763) = -1.0;
	sJ(15764:15962) = -((((var(2402:1:2600)+var(2403:1:2601)).*0.0050251256281407036)./2.0));
	sJ(15963:16161) = -(((var(1)./199.0)./2.0));
	sJ(16162:16360) = -(((var(1)./199.0)./2.0));
	sJ(16361) = 1.0;
	sJ(16362) = 1.0;
	sJ(16363) = 1.0;
	sJ(16364) = 1.0;
	sJ(16365) = 1.0;
	sJ(16366) = 1.0;
	sJ(16367) = 1.0;
	sJ(16368) = 1.0;
	sJ(16369) = 1.0;
	sJ(16370) = 1.0;
	sJ(16371) = 1.0;
	sJ(16372) = 1.0;
	sJ(16373) = 1.0;
	sJ(16374) = 1.0;
	sJ(16375) = 1.0;
	sJ(16376) = 1.0;
	J = sparse(iJ, jJ, sJ, 2408, 2601);

end % ipoptJacobian
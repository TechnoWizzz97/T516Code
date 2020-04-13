function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (27-Mar-2019 16:27:39)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(2637,1) = 0; % Pre-allocation
	c(1:90) = ((var(3:1:92)-var(2:1:91))-(((var(1)./90.0).*(var(93:1:182)+var(94:1:183)))./2.0));
	c(91:180) = ((var(185:1:274)-var(184:1:273))-(((var(1)./90.0).*(var(275:1:364)+var(276:1:365)))./2.0));
	c(181:270) = ((var(549:1:638)-var(548:1:637))-(((var(1)./90.0).*(var(639:1:728)+var(640:1:729)))./2.0));
	c(271:360) = ((var(367:1:456)-var(366:1:455))-(((var(1)./90.0).*(var(457:1:546)+var(458:1:547)))./2.0));
	c(361:450) = ((var(458:1:547)-var(457:1:546))-(((var(1)./90.0).*((((2369.6999999999998.*(var(548:1:637)-var(366:1:455)))-((((101.40000000000001.*var(1094:1:1183))+(var(1185:1:1274).*var(93:1:182)))+(3754.5.*var(184:1:273)))+(0.3145.*var(275:1:364))))./0.008)+(((2369.6999999999998.*(var(549:1:638)-var(367:1:456)))-((((101.40000000000001.*var(1095:1:1184))+(var(1186:1:1275).*var(94:1:183)))+(3754.5.*var(185:1:274)))+(0.3145.*var(276:1:365))))./0.008)))./2.0));
	c(451:540) = ((var(731:1:820)-var(730:1:819))-(((var(1)./90.0).*(var(821:1:910)+var(822:1:911)))./2.0));
	c(541:630) = ((var(822:1:911)-var(821:1:910))-(((var(1)./90.0).*(((((0.008999999999999999.*(101.40000000000001.*var(1094:1:1183)))+((0.008999999999999999.*var(1185:1:1274)).*var(93:1:182)))-((33.790499999999994.*var(184:1:273))+(0.0028304999999999997.*var(275:1:364))))./0.00000032399999999999999)+((((0.008999999999999999.*(101.40000000000001.*var(1095:1:1184)))+((0.008999999999999999.*var(1186:1:1275)).*var(94:1:183)))-((33.790499999999994.*var(185:1:274))+(0.0028304999999999997.*var(276:1:365))))./0.00000032399999999999999)))./2.0));
	c(631:720) = ((var(913:1:1002)-var(912:1:1001))-(((var(1)./90.0).*(var(1003:1:1092)+var(1004:1:1093)))./2.0));
	c(721:810) = ((var(1004:1:1093)-var(1003:1:1092))-(((var(1)./90.0).*((((var(1549:1:1638)-(2369.6999999999998.*(var(548:1:637)-var(366:1:455))))-12.262500000000001)./1.25)+(((var(1550:1:1639)-(2369.6999999999998.*(var(549:1:638)-var(367:1:456))))-12.262500000000001)./1.25)))./2.0));
	c(811:901) = (var(1185:1:1275)-((30.6724./(1.0+exp((100.0.*var(93:1:183)))))+5.4706000000000001));
	c(902:992) = (var(1367:1:1457)-((0.11.*(-(((var(93:1:183)./9.0)--0.08))+(((((var(93:1:183)./9.0)--0.08).^2.0)+0.0121).^0.5)))./(((((var(93:1:183)./9.0)--0.08).^2.0)+0.0121).^0.5)));
	c(993:1083) = (var(1458:1:1548)-(0.01+((((0.11.*(var(93:1:183)./9.0)).*0.5).*((var(93:1:183)./9.0)+((((var(93:1:183)./9.0).^2.0)+0.040000000000000008).^0.5)))./((((var(93:1:183)./9.0).^2.0)+0.040000000000000008).^0.5))));
	c(1084:1174) = (var(1276:1:1366)-(var(1367:1:1457)+var(1458:1:1548)));
	c(1175:1265) = var(366:1:456);
	c(1266:1356) = var(184:1:274);
	c(1357:1447) = var(548:1:638);
	c(1448:1538) = var(2:1:92);
	c(1539:1629) = var(730:1:820);
	c(1630:1720) = ((var(184:1:274)-var(366:1:456))-(0.008999999999999999.*var(730:1:820)));
	c(1721:1811) = ((var(2:1:92)-var(366:1:456))+(0.008999999999999999.*var(730:1:820)));
	c(1812:1902) = var(1094:1:1184);
	c(1903:1993) = var(93:1:183);
	c(1994:2084) = var(275:1:365);
	c(2085:2175) = (var(912:1:1002)-(0.0050000000000000001.*var(1640:1:1730)));
	c(2176:2266) = (0.000025000000000000001-(((0.0050000000000000001.*var(1731:1:1821)).^2.0)+(var(912:1:1002).^2.0)));
	c(2267:2357) = ((((var(1549:1:1639).*0.0025000000000000001).*var(1731:1:1821))+(0.030656250000000003.*var(1731:1:1821)))-(((2369.6999999999998.*(var(548:1:638)-var(366:1:456))).*0.0074999999999999997).*var(1731:1:1821)));
	c(2358:2448) = var(1549:1:1639);
	c(2449:2539) = (2369.6999999999998.*(var(548:1:638)-var(366:1:456)));
	c(2540:2630) = var(912:1:1002);
	c(2631) = var(912);
	c(2632) = var(1003);
	c(2633) = var(1549);
	c(2634) = var(1093);
	c(2635) = (((0.050000000000000003-var(1002))-(var(1093).*(var(1093)./9.81)))-((9.81.*((var(1093)./9.81).^2.0))./2.0));
	c(2636) = var(1639);
	c(2637) = var(1);
end % ipoptConstraints
function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (11-Feb-2019 17:38:20)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(2318,1) = 0; % Pre-allocation
	c(1:100) = ((var(205:1:304)-var(204:1:303))-((var(1)./100.0).*var(305:1:404)));
	c(101:200) = ((var(306:1:405)-var(305:1:404))-((var(1)./100.0).*(((((-(var(2:1:101))+(3000.0.*(0.050000000000000003-var(204:1:303))))+(15.0.*-(var(305:1:404))))-1.9620000000000002)-var(103:1:202))./0.20000000000000001)));
	c(201:300) = ((var(407:1:506)-var(406:1:505))-((var(1)./100.0).*var(507:1:606)));
	c(301:400) = ((var(508:1:607)-var(507:1:606))-((var(1)./100.0).*(var(2:1:101)-9.81)));
	c(401:500) = ((var(609:1:708)-var(608:1:707))-((var(1)./100.0).*var(709:1:808)));
	c(501:600) = ((var(710:1:809)-var(709:1:808))-((var(1)./100.0).*((var(103:1:202)-0.11772000000000001)./0.012)));
	c(601:700) = ((var(1215:1:1314)-var(1214:1:1313))-(((var(1)+var(2022))./100.0).*var(1315:1:1414)));
	c(701:800) = ((var(1316:1:1415)-var(1315:1:1414))-(((var(1)+var(2022))./100.0).*(((-(var(1012:1:1111))-1.9620000000000002)-var(1113:1:1212))./0.20000000000000001)));
	c(801:900) = ((var(1417:1:1516)-var(1416:1:1515))-(((var(1)+var(2022))./100.0).*var(1517:1:1616)));
	c(901:1000) = ((var(1518:1:1617)-var(1517:1:1616))-(((var(1)+var(2022))./100.0).*(var(1012:1:1111)-9.81)));
	c(1001:1100) = ((var(1619:1:1718)-var(1618:1:1717))-(((var(1)+var(2022))./100.0).*var(1719:1:1818)));
	c(1101:1200) = ((var(1720:1:1819)-var(1719:1:1818))-(((var(1)+var(2022))./100.0).*((((3000.0.*(0.050000000000000003-(var(1214:1:1313)-var(1618:1:1717))))+(15.0.*(var(1315:1:1414)-var(1719:1:1818))))-0.49050000000000005)./0.050000000000000003)));
	c(1201:1300) = ((var(1821:1:1920)-var(1820:1:1919))-(((var(1)+var(2022))./100.0).*var(1921:1:2020)));
	c(1301:1400) = ((var(1922:1:2021)-var(1921:1:2020))-(((var(1)+var(2022))./100.0).*((var(1113:1:1212)-0.11772000000000001)./0.012)));
	c(1401:1501) = var(911:1:1011);
	c(1502:1602) = var(810:1:910);
	c(1603:1703) = (var(406:1:506)-var(204:1:304));
	c(1704) = (0.050000000000000003-var(204));
	c(1705) = (((3000.0.*(0.050000000000000003-var(304)))+(15.0.*-(var(405))))-0.49050000000000005);
	c(1706:1806) = (var(204:1:304)-var(810:1:910));
	c(1807:1907) = (-(var(204:1:304))+var(608:1:708));
	c(1908:2008) = (-(var(1214:1:1314))+var(1820:1:1920));
	c(2009) = (var(1315)-var(405));
	c(2010) = (var(1517)-var(607));
	c(2011) = (var(1719)-var(1011));
	c(2012) = (var(1214)-var(304));
	c(2013) = (var(1416)-var(506));
	c(2014) = (var(1618)-var(910));
	c(2015:2115) = (var(1214:1:1314)-var(1618:1:1718));
	c(2116:2216) = (var(1416:1:1516)-var(1214:1:1314));
	c(2217:2317) = var(1618:1:1718);
	c(2318) = var(1314);
end % ipoptConstraints
function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (24-Mar-2020 17:23:20)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(3026,1) = 0; % Pre-allocation
	c(1:150) = ((var(3:1:152)-var(2:1:151))-((var(1)./150.0).*var(153:1:302)));
	c(151:300) = ((var(154:1:303)-var(153:1:302))-((var(1)./150.0).*(var(1814:1:1963)-(0.5.*var(153:1:302)))));
	c(301:450) = ((var(305:1:454)-var(304:1:453))-((var(1)./150.0).*var(455:1:604)));
	c(451:600) = ((var(456:1:605)-var(455:1:604))-((var(1)./150.0).*(var(1965:1:2114)-(0.5.*var(455:1:604)))));
	c(601:750) = ((var(607:1:756)-var(606:1:755))-((var(1)./150.0).*var(757:1:906)));
	c(751:900) = ((var(758:1:907)-var(757:1:906))-((var(1)./150.0).*(var(2116:1:2265)-(0.5.*var(757:1:906)))));
	c(901:1050) = ((var(909:1:1058)-var(908:1:1057))-((var(1)./150.0).*var(1059:1:1208)));
	c(1051:1200) = ((var(1060:1:1209)-var(1059:1:1208))-((var(1)./150.0).*(var(2267:1:2416)-(0.5.*var(1059:1:1208)))));
	c(1201:1350) = ((var(1211:1:1360)-var(1210:1:1359))-((var(1)./150.0).*var(1361:1:1510)));
	c(1351:1500) = ((var(1362:1:1511)-var(1361:1:1510))-((var(1)./150.0).*(var(2418:1:2567)-(0.5.*var(1361:1:1510)))));
	c(1501:1650) = ((var(1513:1:1662)-var(1512:1:1661))-((var(1)./150.0).*var(1663:1:1812)));
	c(1651:1800) = ((var(1664:1:1813)-var(1663:1:1812))-((var(1)./150.0).*(var(2569:1:2718)-(0.5.*var(1663:1:1812)))));
	c(1801) = var(2);
	c(1802) = var(606);
	c(1803) = var(1210);
	c(1804) = var(153);
	c(1805) = var(455);
	c(1806) = var(757);
	c(1807) = var(1059);
	c(1808) = var(1361);
	c(1809) = var(1663);
	c(1810) = var(303);
	c(1811) = var(605);
	c(1812) = var(907);
	c(1813) = var(1209);
	c(1814) = var(1511);
	c(1815) = var(1813);
	c(1816) = var(1360);
	c(1817) = var(1662);
	c(1818) = var(1);
	c(1819:1969) = (var(908:1:1058)-var(304:1:454));
	c(1970:2120) = (var(908:1:1058)-var(1512:1:1662));
	c(2121:2271) = ((var(2720:1:2870).^2.0)-((var(304:1:454).^2.0)+(var(2:1:152).^2.0)));
	c(2272:2422) = ((var(2871:1:3021).^2.0)-(((var(908:1:1058)-var(304:1:454)).^2.0)+((var(606:1:756)-var(2:1:152)).^2.0)));
	c(2423:2573) = ((var(3022:1:3172).^2.0)-(((var(1512:1:1662)-var(908:1:1058)).^2.0)+((var(1210:1:1360)-var(606:1:756)).^2.0)));
	c(2574:2724) = var(2720:1:2870);
	c(2725:2875) = var(2871:1:3021);
	c(2876:3026) = var(3022:1:3172);
end % ipoptConstraints
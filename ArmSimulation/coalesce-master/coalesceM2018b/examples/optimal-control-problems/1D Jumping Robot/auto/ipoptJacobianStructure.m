function [J] = ipoptJacobianStructure(var)
%IPOPTJACOBIANSTRUCTURE
%
% Auto-generated by COALESCE package (11-Feb-2019 17:38:21)
%
% Copyright 2013-2014 Mikhail S. Jones

	iJ = [1:1:100,1:1:100,1:1:100,1:1:100,101:1:200,101:1:200,101:1:200,101:1:200,101:1:200,101:1:200,201:1:300,201:1:300,201:1:300,201:1:300,301:1:400,301:1:400,301:1:400,301:1:400,401:1:500,401:1:500,401:1:500,401:1:500,501:1:600,501:1:600,501:1:600,501:1:600,601:1:700,601:1:700,601:1:700,601:1:700,601:1:700,701:1:800,701:1:800,701:1:800,701:1:800,701:1:800,701:1:800,801:1:900,801:1:900,801:1:900,801:1:900,801:1:900,901:1:1000,901:1:1000,901:1:1000,901:1:1000,901:1:1000,1001:1:1100,1001:1:1100,1001:1:1100,1001:1:1100,1001:1:1100,1101:1:1200,1101:1:1200,1101:1:1200,1101:1:1200,1101:1:1200,1101:1:1200,1101:1:1200,1201:1:1300,1201:1:1300,1201:1:1300,1201:1:1300,1201:1:1300,1301:1:1400,1301:1:1400,1301:1:1400,1301:1:1400,1301:1:1400,1401:1:1501,1502:1:1602,1603:1:1703,1603:1:1703,1704,1705,1705,1706:1:1806,1706:1:1806,1807:1:1907,1807:1:1907,1908:1:2008,1908:1:2008,2009,2009,2010,2010,2011,2011,2012,2012,2013,2013,2014,2014,2015:1:2115,2015:1:2115,2116:1:2216,2116:1:2216,2217:1:2317,2318]';
	jJ = [205:1:304,204:1:303,1+zeros(1,100),305:1:404,306:1:405,305:1:404,1+zeros(1,100),2:1:101,204:1:303,103:1:202,407:1:506,406:1:505,1+zeros(1,100),507:1:606,508:1:607,507:1:606,1+zeros(1,100),2:1:101,609:1:708,608:1:707,1+zeros(1,100),709:1:808,710:1:809,709:1:808,1+zeros(1,100),103:1:202,1215:1:1314,1214:1:1313,1+zeros(1,100),2022+zeros(1,100),1315:1:1414,1316:1:1415,1315:1:1414,1+zeros(1,100),2022+zeros(1,100),1012:1:1111,1113:1:1212,1417:1:1516,1416:1:1515,1+zeros(1,100),2022+zeros(1,100),1517:1:1616,1518:1:1617,1517:1:1616,1+zeros(1,100),2022+zeros(1,100),1012:1:1111,1619:1:1718,1618:1:1717,1+zeros(1,100),2022+zeros(1,100),1719:1:1818,1720:1:1819,1719:1:1818,1+zeros(1,100),2022+zeros(1,100),1214:1:1313,1618:1:1717,1315:1:1414,1821:1:1920,1820:1:1919,1+zeros(1,100),2022+zeros(1,100),1921:1:2020,1922:1:2021,1921:1:2020,1+zeros(1,100),2022+zeros(1,100),1113:1:1212,911:1:1011,810:1:910,406:1:506,204:1:304,204,304,405,204:1:304,810:1:910,204:1:304,608:1:708,1214:1:1314,1820:1:1920,1315,405,1517,607,1719,1011,1214,304,1416,506,1618,910,1214:1:1314,1618:1:1718,1416:1:1516,1214:1:1314,1618:1:1718,1314]';
	sJ = 1 + zeros(1,8431);
	J = sparse(iJ, jJ, sJ, 2318, 2022);

end % ipoptJacobianStructure
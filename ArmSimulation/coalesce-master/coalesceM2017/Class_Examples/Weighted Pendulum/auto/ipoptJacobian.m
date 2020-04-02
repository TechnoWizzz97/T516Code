function [J] = ipoptJacobian(var)
%IPOPTJACOBIAN
%
% Auto-generated by COALESCE package (18-Jan-2019 14:05:32)
%
% Copyright 2013-2014 Mikhail S. Jones

	iJ = [1:1:100,1:1:100,1:1:100,1:1:100,101:1:200,101:1:200,101:1:200,101:1:200,101:1:200,201:1:301,302,303,304,305,306:1:406,306:1:406,407:1:507,407:1:507,508]';
	jJ = [3:1:102,2:1:101,1+zeros(1,100),103:1:202,104:1:203,103:1:202,1+zeros(1,100),2:1:101,305:1:404,305:1:405,2,103,102,203,305:1:405,204:1:304,204:1:304,305:1:405,1]';
	sJ(1410,1) = 0; % Pre-allocation
	sJ(1:100) = 1.0;
	sJ(101:200) = -1.0;
	sJ(201:300) = -((var(103:1:202).*0.01));
	sJ(301:400) = -((var(1)./100.0));
	sJ(401:500) = 1.0;
	sJ(501:600) = (-1.0-((var(1)./100.0).*-0.050000000000000003));
	sJ(601:700) = -(((((16.817142857142859.*(-(cos(var(2:1:101)))-((0.5.*cos(var(2:1:101)))./2.0)))+(var(305:1:404)./0.29166666666666669))-(0.050000000000000003.*var(103:1:202))).*0.01));
	sJ(701:800) = -(((var(1)./100.0).*(16.817142857142859.*(-(-(sin(var(2:1:101))))-((0.5.*-(sin(var(2:1:101))))./2.0)))));
	sJ(801:900) = -(((var(1)./100.0).*3.4285714285714284));
	sJ(901:1001) = 1.0;
	sJ(1002) = 1.0;
	sJ(1003) = 1.0;
	sJ(1004) = 1.0;
	sJ(1005) = 1.0;
	sJ(1006:1106) = 1.0;
	sJ(1107:1207) = 1.0;
	sJ(1208:1308) = 1.0;
	sJ(1309:1409) = -1.0;
	sJ(1410) = 1.0;
	J = sparse(iJ, jJ, sJ, 508, 405);

end % ipoptJacobian
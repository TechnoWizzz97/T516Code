function [c] = ipoptConstraints(var)
%IPOPTCONSTRAINTS
%
% Auto-generated by COALESCE package (10-Apr-2019 12:19:45)
%
% Copyright 2013-2014 Mikhail S. Jones

	c(732,1) = 0; % Pre-allocation
	c(1:90) = ((var(3:1:92)-var(2:1:91))-(((var(1)./90.0).*(var(93:1:182)+var(94:1:183)))./2.0));
	c(91:180) = ((var(94:1:183)-var(93:1:182))-(((var(1)./90.0).*((((var(275:1:364)-var(184:1:273))-12.262500000000001)./1.25)+(((var(276:1:365)-var(185:1:274))-12.262500000000001)./1.25)))./2.0));
	c(181:271) = (var(2:1:92)-(0.0050000000000000001.*var(366:1:456)));
	c(272:362) = (0.000025000000000000001-(((0.0050000000000000001.*var(457:1:547)).^2.0)+(var(2:1:92).^2.0)));
	c(363:453) = (((var(275:1:365).*0.0025000000000000001)+0.030656250000000003)-(var(184:1:274).*0.0074999999999999997));
	c(454:544) = var(275:1:365);
	c(545:635) = var(184:1:274);
	c(636:726) = var(2:1:92);
	c(727) = var(93);
	c(728) = var(275);
	c(729) = var(183);
	c(730) = (0.19620000000000001-(var(183).^2.0));
	c(731) = var(365);
	c(732) = var(1);
end % ipoptConstraints
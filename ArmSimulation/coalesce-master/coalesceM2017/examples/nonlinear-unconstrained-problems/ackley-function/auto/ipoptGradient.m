function [g] = ipoptGradient(var)
%IPOPTGRADIENT
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:35)
%
% Copyright 2013-2014 Mikhail S. Jones

	ig = [1,1]';
	jg = [1,2]';
	sg(2,1) = 0; % Pre-allocation
	sg(1) = ((-20.0.*(exp((-0.20000000000000001.*sqrt((((0.5.*var(1)).*var(1))+((0.5.*var(2)).*var(2)))))).*(-0.20000000000000001.*((((0.5.*var(1))+(var(1).*0.5))./2.0)./sqrt((((0.5.*var(1)).*var(1))+((0.5.*var(2)).*var(2))))))))-(exp((0.5.*cos((((6.2831853071795862.*var(1)).*var(1))+((6.2831853071795862.*var(2)).*var(2)))))).*(0.5.*(-(sin((((6.2831853071795862.*var(1)).*var(1))+((6.2831853071795862.*var(2)).*var(2))))).*((6.2831853071795862.*var(1))+(var(1).*6.2831853071795862))))));
	sg(2) = ((-20.0.*(exp((-0.20000000000000001.*sqrt((((0.5.*var(1)).*var(1))+((0.5.*var(2)).*var(2)))))).*(-0.20000000000000001.*((((0.5.*var(2))+(var(2).*0.5))./2.0)./sqrt((((0.5.*var(1)).*var(1))+((0.5.*var(2)).*var(2))))))))-(exp((0.5.*cos((((6.2831853071795862.*var(1)).*var(1))+((6.2831853071795862.*var(2)).*var(2)))))).*(0.5.*(-(sin((((6.2831853071795862.*var(1)).*var(1))+((6.2831853071795862.*var(2)).*var(2))))).*((6.2831853071795862.*var(2))+(var(2).*6.2831853071795862))))));
	g = sparse(ig, jg, sg, 1, 2);

end % ipoptGradient
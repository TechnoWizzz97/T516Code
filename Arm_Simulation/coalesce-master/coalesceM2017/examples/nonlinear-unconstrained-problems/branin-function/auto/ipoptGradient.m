function [g] = ipoptGradient(var)
%IPOPTGRADIENT
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:23)
%
% Copyright 2013-2014 Mikhail S. Jones

	ig = [1,1]';
	jg = [2,1]';
	sg(2,1) = 0; % Pre-allocation
	sg(1) = (2.0.*(((var(2)-(0.12918450914398066.*(var(1).^2.0)))+(1.5915494309189535.*var(1)))-6.0));
	sg(2) = (((2.0.*(-((0.12918450914398066.*(2.0.*var(1))))+1.5915494309189535)).*(((var(2)-(0.12918450914398066.*(var(1).^2.0)))+(1.5915494309189535.*var(1)))-6.0))+(9.602112642270262.*-(sin(var(1)))));
	g = sparse(ig, jg, sg, 1, 2);

end % ipoptGradient
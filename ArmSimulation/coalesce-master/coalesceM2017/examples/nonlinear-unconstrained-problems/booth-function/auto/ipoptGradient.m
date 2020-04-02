function [g] = ipoptGradient(var)
%IPOPTGRADIENT
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:27)
%
% Copyright 2013-2014 Mikhail S. Jones

	ig = [1,1]';
	jg = [1,2]';
	sg(2,1) = 0; % Pre-allocation
	sg(1) = ((2.0.*((var(1)+(2.0.*var(2)))-7.0))+(4.0.*(((2.0.*var(1))+var(2))-5.0)));
	sg(2) = ((4.0.*((var(1)+(2.0.*var(2)))-7.0))+(2.0.*(((2.0.*var(1))+var(2))-5.0)));
	g = sparse(ig, jg, sg, 1, 2);

end % ipoptGradient
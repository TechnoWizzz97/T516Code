function [g] = ipoptGradient(var)
%IPOPTGRADIENT
%
% Auto-generated by COALESCE package (10-Jan-2019 15:47:28)
%
% Copyright 2013-2014 Mikhail S. Jones

	ig = [1,1+zeros(1,199),1+zeros(1,199)]';
	jg = [1,2:1:200,3:1:201]';
	sg(399,1) = 0; % Pre-allocation
	sg(1) = sum((((var(2:1:200).^2.0)+(var(3:1:201).^2.0))./2.0));
	sg(2:200) = (var(1).*((2.0.*var(2:1:200))./2.0));
	sg(201:399) = (var(1).*((2.0.*var(3:1:201))./2.0));
	g = sparse(ig, jg, sg, 1, 2601);

end % ipoptGradient
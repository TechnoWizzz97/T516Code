function [J] = ipoptJacobianStructure(var)
%IPOPTJACOBIANSTRUCTURE
%
% Auto-generated by COALESCE package (10-Jan-2019 16:49:46)
%
% Copyright 2013-2014 Mikhail S. Jones

	iJ = []';
	jJ = []';
	sJ = 1 + zeros(1,0);
	J = sparse(iJ, jJ, sJ, 0, 2);

end % ipoptJacobianStructure
function [J] = ipoptJacobianStructure(var)
%IPOPTJACOBIANSTRUCTURE
%
% Auto-generated by COALESCE package (10-Apr-2019 12:19:45)
%
% Copyright 2013-2014 Mikhail S. Jones

	iJ = [1:1:90,1:1:90,1:1:90,1:1:90,1:1:90,91:1:180,91:1:180,91:1:180,91:1:180,91:1:180,91:1:180,91:1:180,181:1:271,181:1:271,272:1:362,272:1:362,363:1:453,363:1:453,454:1:544,545:1:635,636:1:726,727,728,729,730,731,732]';
	jJ = [3:1:92,2:1:91,1+zeros(1,90),93:1:182,94:1:183,94:1:183,93:1:182,1+zeros(1,90),275:1:364,184:1:273,276:1:365,185:1:274,2:1:92,366:1:456,457:1:547,2:1:92,275:1:365,184:1:274,275:1:365,184:1:274,2:1:92,93,275,183,183,365,1]';
	sJ = 1 + zeros(1,1905);
	J = sparse(iJ, jJ, sJ, 732, 547);

end % ipoptJacobianStructure
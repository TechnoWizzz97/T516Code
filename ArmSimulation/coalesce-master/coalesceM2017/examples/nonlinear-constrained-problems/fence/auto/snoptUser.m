function [A,iAfun,jAvar,iGfun,jGvar] = snoptUser()
%SNOPTUSER
%
% Auto-generated by COALESCE package (09-Jan-2019 14:20:52)
%
% Copyright 2013-2014 Mikhail S. Jones

	A = [];
	iAfun = []';
	jAvar = []';
	iGfun = [1,2:1:101,2:1:101,2:1:101,2:1:101,2:1:101,102:1:201,102:1:201,102:1:201,102:1:201,202,203,204,205,206]';
	jGvar = [303,204:1:303,203:1:302,2:1:101,1:1:100,103:1:202,2:1:101,1:1:100,103:1:202,102:1:201,1,102,101,202,203]';
end % snoptUser
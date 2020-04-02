function T = makeTF(this, th, axis, tx, ty, tz)
%ROTATE Apply transformation operation on graphics model object.
%
% Description:
%   Creates a transformation matrix with some translation and rotation
%	Axis args:   'X'  |  'Y'  |  'Z'

	defined = false;

	% Rotation about x-axis transformation matrix
	if strcmp(axis, 'X')
		cx = cos(th); sx = sin(th);
		T = [
				1		0		0		tx;
			  	0		cx		-sx		ty;
			  	0		sx		cx		tz;
			  	0		0		0		1];
		defined = true;
	end

	% Rotation about y-axis transformation matrix
	if strcmp(axis, 'Y')
		cy = cos(th); sy = sin(th);
		T = [
			cy		0		sy		tx;
			0		1		0		ty;
		  	-sy		0		cy		tz;
		  	0		0		0		1];
		defined = true;
	end

	% Rotation about z-axis transformation matrix
	if strcmp(axis, 'Z')
		cz = cos(th); sz = sin(th);
		T = [
			cz		-sz		0		tx;
			sz		cz		0		ty;
			0		0		1		tz;
			0		0		0		1];
		defined = true;
	end
	% Final rotation transformation
	% R = Rz*Ry*Rx;

	if ~defined
		disp('---- ERROR ---- Transformation not defined')
	end
	% Apply rotation to transformation matrix
  % this.A = T*this.A;
end % transformation

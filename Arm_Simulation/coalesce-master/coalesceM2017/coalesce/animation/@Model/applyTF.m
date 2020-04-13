function T = applyTF(this, T)
%ROTATE Apply transformation operation on graphics model object.

	% Apply rotation to transformation matrix
   this.A = T*this.A;
end % transformation

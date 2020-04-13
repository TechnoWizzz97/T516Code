classdef SquishBlock < Model
%SQUISHBLOCK Creates a 3D squishblockmodel object.
%
% Description:
%   Creates a 3D squish block object using the Model superclass.

	properties
		radius@double scalar = 0.1
		length@double scalar = 1
        length0@double scalar = 1;
		width@double scalar = 0.25
		color = 'w'
		nNodes@double scalar = 50
	end % properties

	methods
		function this = SquishBlock(radius, length, width, color, nNodes)
		%SQUISHBLOCK Creates a 3D squishblockmodel object.
		%
		% Syntax:
		%   obj = SquishBlock
		%   obj = SquishBlock(radius, length, thickness)
		%   obj = SquishBlock(radius, length, thickness, color)
		%   obj = SquishBlock(radius, length, thickness, color, nNodes)

			% Call superclass constructor
			this = this@Model;

			% Parse input arguments and set default properties
			switch nargin
				case 0
					% Do nothing
				case 3
					this.radius = radius;
					this.length = length;
                    this.length0 = length;
					this.width = width;
				case 4
					this.radius = radius;
					this.length = length;
                    this.length0 = length;
					this.width = width;
					this.color = color;
				case 5
					this.radius = radius;
					this.length = length;
                    this.length0 = length;
					this.width = width;
					this.color = color;
					this.nCoils = nNodes;
				otherwise
					error('Invalid number of input arguments.');
			end % switch

            % Check radius size is appropriate
			this.radius = min([this.radius, this.width/2, this.length/2]);
			if this.radius <= 0
				error('Radius must be positive real number.');
			end % if
            
            this = this.setLength(length);
            
			% Create graphics object
			this.handle = patch(this.x, this.y, this.z, ...
				'EdgeColor', 'k', ...
				'FaceColor', this.color);
		end % Spring

		function ret = setLength(this, length)
		%SETLENGTH Set squishblock length
		%

			% Set property
			this.length = length;

            % Parametric equations for a rounded square
            theta = (0:this.nNodes)/this.nNodes*2*pi;
			this.x = this.radius*sin(theta);
			this.y = this.radius*cos(theta);
			this.z = 0.*theta;
			this.x(this.x > 0) = this.x(this.x > 0) + this.length - 2*this.radius;
			this.y(this.y > 0) = this.y(this.y > 0) + this.width-(this.length-this.length0)/5 - 2*this.radius;
			this.x = this.x + this.radius;
			this.y = this.y + this.radius;
            
            if nargout > 0
                ret = this;
            end
		end % setLength
	end % methods
end  % classdef

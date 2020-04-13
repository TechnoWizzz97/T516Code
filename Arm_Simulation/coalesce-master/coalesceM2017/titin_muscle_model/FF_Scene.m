classdef FF_Scene < Scene
    
    properties
        % Geometry
        Lf@double
        Lh@double
        R@double
        Ls@double
        
        axes
        response
        it
        
        % Shape Type
        Mass@Circle
        Foot@RoundedSquare
        Shank@RoundedSquare
    end
    
    methods
        %Constructor
        function obj = FF_Scene(geo, response)
            obj = obj@Scene(response.time);
            
            obj.response = response;
            
            % System Parameters
            obj.Lf = geo.Lf;
            obj.Lh = geo.Lh;
            obj.R = geo.R;
            obj.Ls = geo.Ls;
        end
        
        function obj = initialize(obj)
            
            obj.axes = gca;
            obj.it = 1;
            
            obj.Mass = Circle(obj.R/2, 'b');
            obj.Foot = RoundedSquare(obj.Lf+obj.Lh, obj.R/30, obj.R, 'k');
            obj.Shank = RoundedSquare(obj.Ls, obj.R/40, obj.R, 'k');
          
            axis([-obj.R obj.R 0 obj.Ls+obj.R*3]);
            view(0,89.9);
            axis on;
        end 
        
        function obj = update(obj, t)

            iter = obj.it;
            
            y = obj.response.yf(iter, :);
            phi = asin(obj.response.sphi(iter, :));
            
            
            obj.Mass.reset;
    %        obj.Mass.translate(obj.Lf*obj.response.cphi(iter,:), obj.Ls, 0);
            obj.Mass.translate(0, obj.Ls, 0);
            obj.Mass.translate(0, y, 1);
            obj.Mass.update;            
            
            obj.Foot.reset;
            obj.Foot.rotate(0,0, phi);
            obj.Foot.translate(-obj.Lf*obj.response.cphi(iter,:), 0,0);
            if iter >= obj.response.modeswitch
                obj.Foot.translate(0, y-obj.Lf*sin(phi), 0);
            end
            obj.Foot.update;
            
            obj.Shank.reset;
            obj.Shank.rotate(0,0,pi/2);
         %   obj.Shank.translate(obj.Lf*obj.response.cphi(iter,:), 0,0);
            obj.Shank.translate(0, y, 0);
            obj.Shank.update;

            obj.it = obj.it + 1;
            if obj.it > obj.response.maxiter
                obj.it = 1;
            end
        end
        
    end
    
end
            
            
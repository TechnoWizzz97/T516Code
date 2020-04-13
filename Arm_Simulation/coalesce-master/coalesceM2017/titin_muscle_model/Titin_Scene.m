classdef Titin_Scene < Scene
    
    properties
        % Geometry
        Lm@double
        R@double
        Lp@double
        Lce@double
        Lts@double
        
        axes
        response
        it
        
        % Shape Type
        Pulley@Circle
        Theta@RoundedSquare
        CE@SquishBlock
        TS@SquishBlock
        Base@StripedLine
        M@Circle
        SEE@Spring
    end
    
    methods
        %Constructor
        function obj = Titin_Scene(geo, response)
            obj = obj@Scene(response.time);
            
            obj.response = response;
            
            % System Parameters
            obj.Lm = geo.L;
            obj.R = geo.R;
            obj.Lp = geo.L/2;
            obj.Lce = geo.L/4;
            obj.Lts = geo.L/4;
        end
        
        function obj = initialize(obj)
            
            obj.axes = gca;
            obj.it = 1;
            
            obj.Base = StripedLine(obj.Lm, obj.Lm/20);
            obj.Pulley = Circle(obj.R/2, 'b');
            obj.Theta = RoundedSquare(obj.R/2, obj.R/40, obj.R, 'k');
            obj.CE = SquishBlock(obj.R/16, obj.Lce/2, obj.Lm/8, 'r');
            obj.TS = SquishBlock(obj.R/16, obj.Lts/2, obj.Lm/8, 'g');
            obj.M = Circle(obj.R/4, 'k');
            obj.SEE = Spring(obj.R/10, obj.Lm/2, obj.R/80','k',10);
            
            axis([-obj.Lm/8 obj.Lm*1.5 -0.05 0.05]);
            view(0,89.9);
            axis off;
        end 
        
        function obj = update(obj, t)

            iter = obj.it;
            
            xp = obj.response.xp(iter, :);
            xm = obj.response.xm(iter, :);
            xce = obj.response.xce(iter, :);
            xts = obj.response.xts(iter, :);
            theta = obj.response.theta(iter, :);
            
            obj.Base.reset;
            obj.Base.rotate(0, 0, -pi/2);
            obj.Base.translate(0, obj.Lm/2, 0);
            obj.Base.update;
            
            obj.Pulley.reset;
            obj.Pulley.rotate(0, 0, theta);
            obj.Pulley.translate(obj.Lm/2, 0, .01);
            obj.Pulley.translate(xp, 0, 0);
            obj.Pulley.update;
            
            obj.Theta.reset;
            obj.Theta.rotate(0, 0, theta);
            obj.Theta.translate(obj.Lm/2, 0, .02);
            obj.Theta.translate(xp, 0, 0);
            obj.Theta.update;
            
            obj.CE.reset;
            obj.CE.setLength(obj.Lm/2+xce);
            obj.CE.translate(0, obj.R/2-obj.Lce/8, 0);
            obj.CE.update;
            
            obj.TS.reset;
            obj.TS.setLength(obj.Lm/2+xts);
            obj.TS.translate(0, -obj.R/2-obj.Lts/8, 0);
            obj.TS.update;
            
            obj.M.reset;
            obj.M.translate(obj.Lm, obj.R/28, 0);
            obj.M.translate(xm, 0, 0);
            obj.M.update;
            
            
            obj.SEE.reset;
            obj.SEE.setLength(obj.Lm/2 + xm-xp);
            obj.SEE.translate(obj.Lm/2 + xp, obj.R/28, 0);
            
            obj.SEE.update;
            
            obj.it = obj.it + 1;
            if obj.it > obj.response.maxiter
                obj.it = 1;
            end
            
        end
        
    end
    
end
            
            
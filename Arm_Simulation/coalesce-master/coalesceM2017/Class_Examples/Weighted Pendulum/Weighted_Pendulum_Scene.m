

classdef Weighted_Pendulum_Scene < Scene
    
    properties
        r@double  % Link 1 total Length
        d@double % Weight Diameter
        
        axes
        weight@Circle
        base@RoundedSquare
        r1@RoundedSquare
        
        response@Response
    end
    
    methods
        
        % Scene constructor
        function obj = Weighted_Pendulum_Scene(sys, response)
            
            % Call superclass scene constructor
            obj = obj@Scene(response.time{1});
            
            % Store computed system response
            obj.response = response;
            
            % store system parameters
            obj.r = sys.r;
            obj.d = sys.d;
        end
        
        % Initialize graphical objects' Locations and Dimensions
        function obj = initialize(obj)
        obj.axes = gca;
        
        L = obj.r;
        D = obj.d;
        
        obj.base = RoundedSquare(L/10, L/10, L/40);
        obj.base.translate(-obj.base.width/2, -obj.base.height/2, 0);
        obj.base.update;
        
        obj.r1 = RoundedSquare(obj.r + L/20, L/20, L/40, 'r');
        
        obj.weight = Circle(D/2, 'b');        
        
        view(0, 89.9);
        axis off;
        end
        
        function obj = update(obj, t)
            
            [x, u] = obj.response.eval(t);
            
            obj.r1.reset;
            obj.r1.translate(-obj.r1.height/2, -obj.r1.height/2, 0);
            obj.r1.rotate(0, 0, x(1));
            obj.r1.translate(0, 0, 1e-3);
            obj.r1.update;
            
            obj.weight.reset;
            obj.weight.translate(obj.r*cos(x(1)), obj.r*sin(x(1)), 0);
            obj.weight.translate(0,0,1e-3);
            obj.weight.update;
            
            yLim = 1.1*(obj.r + obj.d);
            xLim = yLim/720*1280;
            ylim(obj.axes, [-yLim, yLim]);
            xlim(obj.axes, [-xLim, xLim]);
            
        end
        
    end
end
            
      
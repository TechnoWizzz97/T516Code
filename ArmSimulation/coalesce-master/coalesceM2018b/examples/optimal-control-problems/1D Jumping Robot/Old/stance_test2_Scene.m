classdef stance_test2_Scene
    properties
  	h0@double % Lower height limit
  	hf@double % Upper height limit
  	uMax@double % Max thrust

  	axes%@double
  	ground@StripedLine
  	spring@Spring
    rod@RoundedSquare
    motor@RoundedSquare
    
    response@Response
    end
    
    methods
        function obj = stance_test2_Scene(sys, response)
            
            obj = obj@Scene(response.time{1});
            
            obj.response = response;
            
            % Store parameters
			obj.h0 = response.states{1}(1,1);
			obj.hf = response.states{1}(end,1);
			obj.uMax = max(response.inputs{1});
		end % LunarLanderScene
        
        function obj = initialize(obj)
		%INITIALIZE Initialize graphical objects.
        
			obj.axes = gca;
            
            l=obj.spring.length
            obj.spring = Spring(0.25,l)
            
            obj.rod = RoundedSquare(1,.25,0.1);
            obj.motor = RoundedSquare(0.25, .5,0.1)
            
            obj.ground = StripedLine(10, 0.125);
			
            
            	% Axes properties
			view([0, 89.9]); axis off;
		end % initialize
        
        function obj = update(obj,t)
            
            % Evaluate response at current time
			[x, u] = obj.response.eval(t);

			% Normalize thruster
			u = u/obj.uMax;
            
            obj.spring.reset
            obj.spring = Spring(0.25,l)
            
            obj.rod = RoundedSquare(1,.25,0.1);
            obj.motor = RoundedSquare(0.25, .5,0.1)
            
            
            

            
            
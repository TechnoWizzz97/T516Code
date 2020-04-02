classdef stance_test2_system < SecondOrderSystem
    
    
    properties
        mr=10;%mass of rod
        mm=10;%mass of motor
        k=1;%spring const
        b=1;%damping const
        g=9.81;%gravity
        lo=.5; %natural length of spring
        lr=1%length of rod
        rr=.5;% rod distance to COM
        fLim= 5;
        
    end
    
    %public methods
    methods
        function obj = stance_test2_system
            obj = obj@SecondOrderSystem({'z'},{'f'},{''});
            obj.inputLowerBounds = -obj.fLim;
            obj.inputUpperBounds = obj.fLim;
            
        end
        
        function[M,f]= secondOrderStateEquation(obj, t, x, xdot, u, m)
        M=obj.m,
        f = u - obj.g;
		end % secondOrderStateEquation
	end % methods
end % classdef

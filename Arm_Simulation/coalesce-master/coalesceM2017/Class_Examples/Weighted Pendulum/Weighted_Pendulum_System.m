

classdef Weighted_Pendulum_System < SecondOrderSystem
    
    properties (Constant = true)
        mb = 1                      % [kg] mass of ball
        ma = 0.5                   % [kg] mass of arm
        r = 0.5                       % [m] Lengh of arm
        g = 9.81                    % [m/s2] Gravitational constant
        d = 0.1                      % [m] Weight Diameter
        lb = .25                     % [kg*m2] Moment of ball abt center of rotation
        Ia = .04167               % [kg*m2] Moment of arm abt center of rotation
        I = 0.29167               % [kg*m2] Total Moment on base
        b = 0.05                    % Damping of rotational component
        tau_limit = 10           % Torque Limit
    end
    
    properties (Constant = false)
%         lb = mb*r^2             % [kg*m2] Moment of ball abt center of rotation
%         Ia = 1/3*ma*r^2      % [kg*m2] Moment of arm abt center of rotation
%         I = Ia+Ib                    % [kg*m2] Total Moment acting on hinge pt
    end
    
    methods
        function obj = Weighted_Pendulum_System
            
            obj = obj@SecondOrderSystem({'theta'},{'tau'},{''});
            
            obj.inputLowerBounds = -obj.tau_limit;
            obj.inputUpperBounds = obj.tau_limit;
            
        end
        
        function [M, f] = secondOrderStateEquation(obj, t, x, xdot, u, m)
            
            M(1,1) = obj.I;    
            
           f = obj.r*obj.g/obj.I*(-obj.mb*cos(x)-obj.ma*cos(x)/2)+u/obj.I -obj.b*xdot;
            
        end
            
    end
end
            
            
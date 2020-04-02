classdef PF_Scene < Scene
    
    properties
        % Geometry
        Lpf@double  %Plantarflexor
        Ls@double    % Shank
        Lma@double      % Moment arm ( heel-to-ankle )
        Lf@double     % Forefoot ( ankle-to-toe )
        Lfp@double   % Footprint
        psi@double    % Toe Angle
        gam@double  % Shank Angle ( vertical )
        th_init@double       % Init Ankle Angle
        ya_init@double      % Initial ankle - y
        phi_init@double     % Initial PF Angle
        alpha@double        % Foot Arch Angle
        eps@double            % Heel Angle
     %   beta@double          % pf-shank angle
        
        axes
        response
        it
        
        % Shape Type
        shank@RoundedSquare
        ma@RoundedSquare
        forefoot@RoundedSquare
        ground@StripedLine
        pf@Spring
    end
    
    methods
        % Constructor
        function obj = PF_Scene(geo, response)
            obj = obj@Scene(response.time);
            
            obj.response = response;
            
            % store system parameters
            obj.psi = geo.psi;
            obj.gam = response.gamma(1);
            obj.Ls = geo.Ls;
            obj.Lma = geo.Lma;
            obj.Lf = geo.Lf;
            obj.Lfp = geo.Lfp;
            obj.alpha = geo.alpha;
            obj.eps = pi - geo.alpha - geo.psi;
            obj.th_init = geo.th0;
            obj.ya_init = geo.ya0;
            obj.phi_init = geo.phi0;
        end
        
        function obj = initialize(obj)
            
            obj.axes = gca;
            obj.it = 1;
            
            pfL= obj.response.Lpf(1);
            shankL = obj.Ls;
            heelL = obj.Lma;
            footL = obj.Lf;
            initY = obj.ya_init;
            phi0 = obj.phi_init;
            
            obj.shank = RoundedSquare(shankL,footL/20,shankL/40);
       %     obj.pf = Spring(footL/20, shankL, .0025, 'r', 10);
            obj.forefoot = RoundedSquare(footL, footL/20, shankL/2);
            obj.ma = RoundedSquare(heelL, heelL/4, shankL/40);
            obj.ground = StripedLine(1, 0.025);
            
            view(0, 89.9);
            axis off;
        end  % initialize
        
        function obj = update(obj, t)
            
            iter = obj.it;
            
            ankle = obj.response.ankle(iter, :);
            knee = obj.response.knee(iter, :);
            theta = obj.response.theta(iter);
            pfL =obj.response.Lpf(iter);
            phi = obj.response.phi(iter);
            gamma = obj.response.gamma(iter);
            delth = obj.response.delth(iter);
            beta = obj.response.beta(iter);
            
            obj.ground.reset;
            obj.ground.update;
            
            obj.forefoot.reset;
            obj.forefoot.rotate(0,0,delth+obj.psi);
            obj.forefoot.update;
       
            obj.ma.reset;
            obj.ma.rotate(0,0,delth-obj.psi+obj.alpha);
            obj.ma.translate(obj.Lfp*cos(delth+.025), obj.Lfp*sin(delth+.025), 0);
            obj.ma.translate(0, 0, 1e-3);
            obj.ma.update;
            
            obj.shank.reset;
            obj.shank.rotate(0, 0, theta);
            obj.shank.translate(obj.Lf*cos(delth+obj.psi+.025), obj.Lf*sin(delth+obj.psi+.025), 0);
            obj.shank.translate(0.0095, 0, 0);
            obj.shank.update;
            
%             obj.pf.reset;
% %             obj.pf.rotate(0, 0, delth+(180-phi));
% %             obj.pf.translate(obj.Lfp*cos(delth+.025), obj.Lfp*sin(delth+.025), 0);
%             obj.pf.rotate(0,0, delth-pi/2-beta-gamma);
%             obj.pf.translate(obj.Lf*cos(delth)+obj.Ls*cos(theta), ...
%                 obj.Lf*sin(delth)+obj.Ls*sin(theta), 0);
%             obj.pf.update;
            
            obj.it = obj.it + 1;
            if obj.it > obj.response.maxiter
                obj.it = 1;
            end
        end  % update
        
    end
end

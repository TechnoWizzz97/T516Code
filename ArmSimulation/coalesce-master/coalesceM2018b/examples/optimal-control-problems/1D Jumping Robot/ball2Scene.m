classdef ball2Scene < Scene
    
    % public properties
   properties 
    l1@double % Link 1 total length
  	l2@double % Link 2 total length
    
    
    axes
%   	groundleft@StripedLine
%   	groundright@StripedLine
    
    ground@StripedLine
    mass@RoundedSquare
  	rod@RoundedSquare
    rotor@RoundedSquare

    spring@Spring
    
    response;
   end
   
   
   % public methods
   methods
       function obj = ball2Scene(response);
           
           %this calls the superclass constuctor
           obj = obj@Scene(response.time{1});
           
           %this stores the system response
           obj.response = response;
           
           const_var= response.const;
           hb=const_var(1);
           lo=const_var(2);
           hp=const_var(3);
           lp=const_var(4);
           
           
       end
       
       function obj = initialize(obj)
           
           %store axes handle
           obj.axes = gca;
           
           %set colors
           GROUND_COLOR = [0.5 1 0.5]; %[0.9 0.9 0.6];
           JUMPER_COLOR = [0.85 0.85 0.85];
           MASS_COLOR = [0.85 0.85 0.85];

% % % %            %Define ground
% % % %             obj.ground = StripedLine(2, 0.5/80);
% % % % 			obj.ground.scale(-1, 1, 1);
% % % %             obj.ground.translate(1, -0.5/80 + 0.002, -1);
% % % % 			obj.ground.update;
% % % % 
% % % %            % Define mass
% % % %             obj.mass = RoundedSquare(1, 0.25, 0.02/10,MASS_COLOR);
% % % %             obj.mass.translate(-1/2, 5, 0);
% % % % 			obj.mass.update;
% % % % 
% % % % 			% Define rod
% % % %             obj.rod = RoundedSquare(0.25, 1, 0.01/10,JUMPER_COLOR);
% % % %             obj.rod.translate(-0.25/2, 5, 0);
% % % % 			obj.rod.update;
% % % %             
% % % %              
% % % %             %Define spring
% % % %             coils= 6;
% % % %             obj.spring = Spring(0.125, 5, .005, 'k',coils);
% % % %             obj.spring.rotate(0,0,pi/2)
% % % %             obj.spring.translate(-0.125/2, -0.0005/2, 0);
% % % % 			obj.spring.update;

           const_var= obj.response.const;
           hb=const_var(1);
           lo=const_var(2);
           hp=const_var(3);%height of rotor
           lp=const_var(4); %length of rotor

           %Define prop
           obj.rotor = RoundedSquare(lp,hp);
           obj.rotor.translate((-1/2)*lp,lo+hb,0);
           obj.rotor.update;
           
           %Define ground
            obj.ground = StripedLine(2, 0.5/80);
			obj.ground.scale(-hb*2, hb*2, hb*2);
            obj.ground.translate(hb*2, -0.5/80 + 0.002, -hb*2);
			obj.ground.update;

           % Define mass
            obj.mass = RoundedSquare(hb, hb*0.25, 0.02/10,MASS_COLOR);
            obj.mass.translate(-1/2*hb, lo, 0);
			obj.mass.update;

			% Define rod
            obj.rod = RoundedSquare(0.25*hb, hb, 0.01/10,JUMPER_COLOR);
            obj.rod.translate(-0.25*hb/2, lo, 0);
			obj.rod.update;
             
            %Define spring
            coils= 6;
            obj.spring = Spring(lo/10, lo, lo/15, 'k',coils);
            obj.spring.rotate(0,0,pi/2)
            obj.spring.translate(-lo/20, -0.0005/2, 0);
			obj.spring.update;
       end
      
       function obj = update(obj,t)
           
          response = obj.response;
          
           const_var= obj.response.const;
           hb=const_var(1);
           lo=const_var(2);
           hp=const_var(3);
           lp=const_var(4);
           
%        keyboard
            zm = interp1(response.time{1}, response.zm, t);
			zb = interp1(response.time{1}, response.zb, t);
			zf = interp1(response.time{1}, response.zf, t);
            zg = interp1(response.time{1}, response.zg, t);
            zrot = interp1(response.time{1}, response.zrot, t);
%        keyboard


            % Update Rotor/prop
            obj.rotor.reset
            obj.rotor.translate((-1/2)*lp,zrot,0);
            obj.rotor.update;
            
            % Update mass
			obj.mass.reset; %sets mass back to starting location (0,0)
			obj.mass.translate(0, zm, 0);
			obj.mass.translate(-hb*1/2, 0, 2e-3);
			obj.mass.update;

			% Update rod
			obj.rod.reset;%sets rod back to starting location (0,0)
			obj.rod.translate(0, zb, 0);
			obj.rod.translate(-0.25*hb/2, 0, 1e-3);
			obj.rod.update;

			% Define spring
			obj.spring.reset; %sets spring back to starting location (0,0)
            obj.spring.rotate(0,0,pi/2);
            obj.spring.translate(-lo/20/2, zf, 0);
            obj.spring.setLength(zb-zf)
            obj.spring.update;
            
            
            %axes limits ( maxes animation axes stay constant)
            ylim(obj.axes, [-0.25, max(response.zrot)+0.5]);
            xlim(obj.axes, [-0.25,.25]);
            
       end %updates
   end %methods
end%classdef
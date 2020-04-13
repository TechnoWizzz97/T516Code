
	
%% WFM_v9_1 Integral Method
% Jeremy Petak & Christopher D. Whitney
% Created: 2/23/2016
% July 27th 2016: Theta is multipled by the activation to prevent the build
% up of displacement in the CE and the titn.
%
 
function [simulated] = wfm9(data,params)
 
Length = data.length;
Size = length(data.length);
Act = data.act * params.act_factor;
delta_t = data.time(2) - data.time(1);
 
%Free parameters
kss = params.kss;
kts = params.kts;
M = params.M;
 
 
Cce_L = params.Cce_L;
Cce_S = params.Cce_S;
%Cts = params.wfm.Cts;
Cts_L = params.Cts_L;
Cts_S = params.Cts_S;
 
Po = params.Po;
Lo = params.Lo;
R = Lo/2;
J = 0.5*params.M*R^2;
T_act = params.T_act;
%% Matrix length presizing
Xm = zeros(Size,1);
Xp = zeros(Size,1);
Xdotp = zeros(Size,1);
Xdotm = zeros(Size,1);
Xts = zeros(Size,1);
Xdotts = zeros(Size,1);
Xddotp = zeros(Size,1);
Xce = zeros(Size,1);
Xdotce = zeros(Size,1);
Xss = zeros(Size,1);
Thetap = zeros(Size,1);
Thetadotp = zeros(Size,1);
Thetaddotp = zeros(Size,1);
Fce = zeros(Size,1);
Fm = zeros(Size,1);
Clutch = ones(Size,1);
Fm = zeros(Size,1);
Fts = zeros(Size,1);
Ftd = zeros(Size,1);
Fcd = zeros(Size,1);
Fv = zeros(Size,1);
Active_FL = zeros(Size,1);
Passive_Fl = zeros(Size,1);
%% Initial Conditions
% I feel like this is unnessacry since the
% matrix are intilized to zero already - Chris (June 29,2016)
 
%Lengths
Xp(1) = 0;
Xts(1) =  0;
Xce(1) = 0;
Xm(1) = 0;
%Xdiff(1) = 0;
Thetap(1) = 0;
%Velocities
Xdotm(1) = 0;
Xdotp(1) = 0;
Xdotts(1) = 0;
Xdotce(1) = 0;
Xddotp(1) = 0;
Thetadotp(1) = 0;
 
%Forces
Fce(1) = 0;
Fm(1) = 0;
 
%% Force Calculation Loop
for i= T_act:Size %trying i starting at 2 combined with i-1 for if statement 4/28/15
  %%X_m (muscle length) and Xdot_m (muscle velocity)
  Xm(i) = Length(i);
  %Xm(i) = Xm(i-1) + (Length(i)-Length(i-1)); %changed to Xm(i-1)+Length i and i-1 5/8/15
  Xdotm(i) = (Length(i)-Length(i-1))/delta_t; %changed to i and i-1 4/28/15
 
  Active_Fl(i) = (-878.25 * ( (Length(i)/Lo)  * 1.253)^2 + 2200.4 * ((Length(i)/Lo)*1.254) - 1192) / 186.24;
 
  % Passive FL as function of only length
  %Passive_Fl(i) = exp(p1 + (p2*Length(i))) / p3;
  %Passive_Fl(i) = 0;
 
  %Fce(i) = Po*Active_Fl(i)*Act(i-(T_act-1));
  Fce(i) = Po*Act(i-(T_act-1));
   
  if Xdotts(i-1) > 0
    Cts = Cts_L;
  else
   Cts = Cts_S;
  end
 
  if Xdotce(i-1) > 0
      Cce = Cce_S;
  else
      Cce = Cce_L;
  end
 
  Xddotp(i) = (kss*(Xm(i-1) - Xp(i-1)) - (Fce(i-1)+ Cce*Xdotce(i-1) + kts*Xts(i-1) + Cts*Xdotts(i-1)))/M;
  Xdotp(i) = Xddotp(i)*delta_t + Xdotp(i-1);
  Xp(i) =  (Xddotp(i)/2)*delta_t^2 + Xdotp(i-1)*delta_t + Xp(i-1);
 
  %if Thetadotp(i-1) < 0 && Xdotp(i-1) < 0
  %   Thetap(i) = Thetap(i-1);
  %  Clutch(i) = 2;
 
  %else %Open system
      Thetaddotp(i) = ((R*Fce(i-1) + R*Cce*Xdotce(i-1)) - (R*kts*Xts(i-1) + R*Cts*Xdotts(i-1)))/J;
      Thetadotp(i) = Thetaddotp(i)*delta_t + Thetadotp(i-1);
      Thetap(i) = (Thetaddotp(i)/2)*delta_t^2 + Thetadotp(i-1)*delta_t + Thetap(i-1);
      Clutch(i) = 1;
  %end
 
  %Combined
  Xts(i) = Xp(i) + R*Thetap(i);
  Xdotts(i) = Xdotp(i) + R*Thetadotp(i);
 
  Xce(i) = Xp(i) - R*Thetap(i);
 
  Xdotce(i) = Xdotp(i) - R*Thetadotp(i);
  Xss(i) = Xm(i)-Xp(i);
  % Stops the pulley position from going negative
  if Xm(i)-Xp(i) < 0
      Xp(i) = Xm(i);
  end
  if Xce(i) > 0
    Xce(i) = 0;
  end
 
  %Passive_Fl(i) = Passive_Fl(i)+5;
  %Property of series springs, series spring forces are eqaul to the
  %forces of the contractile element and titin spring
  Fm(i) = kss*Xss(i);
 
  Fts(i) = kts*Xts(i);
  Ftd(i) = Cts*Xdotts(i);
  Fcd(i) = Cce*Xdotce(i);
 
end %End of for loop
 
% adjust forces for penation_angle
Fm = Fm ./ cos(params.penation_angle);
%Fm = Fm .+ 10;
 
%%TO-DO: Restructe these assignments into the header of the model.
 
%% - Chris D. Whitney (Nov. 12th, 2016)
%% Assign output vectors to simulated struc
% Displacement
simulated.positions.Xm = Xm;
simulated.positions.Xp = Xp;
simulated.positions.Xts = Xts;
simulated.positions.Thetap = Thetap;
simulated.positions.Xce = Xce;
simulated.positions.Xss = Xss;
% Velcocitys
simulated.velocitys.Xdotm = Xdotm;
simulated.velocitys.Xdotce = Xdotce;
simulated.velocitys.Xdotts = Xdotts;
simulated.velocitys.Xdotp = Xdotp;
simulated.velocitys.Thetadotp = Thetadotp;
% Accelerations
simulated.accelerations.Xddotp = Xddotp;
simulated.accelerations.Thetaddotp = Thetaddotp;
% Forces
simulated.forces.Fm = Fm;
simulated.forces.Fts = Fts;
simulated.forces.Ftd = Ftd;
simulated.forces.Fce = Fce;
simulated.forces.Fcd = Fcd;
% Relations
simulated.relations.Fv = Fv;
simulated.relations.Active_Fl = Active_Fl;
simulated.relations.Passive_Fl = Passive_Fl;
 
simulated.Clutch = Clutch;
 
end

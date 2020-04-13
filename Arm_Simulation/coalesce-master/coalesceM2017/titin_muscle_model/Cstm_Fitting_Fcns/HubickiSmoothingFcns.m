% MuscleFunctionTesting

clc
clear
close all


v = linspace(-2,2,5e5);
G = 0.17;
vmax = 0.326;

e = 0.01;

fp = (1-v/vmax)./(1+v/(G*vmax));
fn = 1.8-0.8*(1+v/vmax)./(1-7.56*v/(G*vmax));

% fpReg = (1-v)./(1+sqrt(v.^2+e^2)/(G+vmax));
% fnReg = (1+v)./(1+7.56*sqrt(v.^2+e^2)/(G+vmax));

% regP = 0.5.*(v+sqrt(v.^2+e^2))./sqrt(v.^2+e^2);
% regN = 0.5.*(-v+sqrt(v.^2+e^2))./sqrt(v.^2+e^2);
regP = 0.5*(v./sqrt(v.^2+e^2)+1);
regN = 0.5*(-v./sqrt(v.^2+e^2)+1);

fp_reg = fp.*0.5.*(v+sqrt(v.^2+e^2));

vabs = sqrt(v.^2+e^2);
f_reg = 0.5*(G*vmax)*( ...
	(1+v/vmax)./(G*vmax+vabs).*(1-v./vabs) + ...
	(1.8/(G*vmax)-0.8*(1-v/vmax)./(G*vmax+7.56*vabs)).*(1+v./vabs));

figure
hold on
handle1 = title('F-V Relationship (contractile element)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = xlabel('CE velocity (m/s)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = ylabel('Force amplification');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
% plot(v,fn,'r--',v,fp,'b--','LineWidth',2)
plot(v,f_reg,'k-')
axis([-vmax,vmax,0,2])
hold off


% figure
% hold on
% plot(v(2:end),diff(f_reg)/(v(2)-v(1)),'k-')
% axis([-0.6,0.6,-50,50])
% hold off


l = linspace(0,1,1000);
l_ce0 = 0.04;

a_act = 3.19;
b_act = 0.87;
s_act = 0.39;
f_lce_act = exp(-abs(((l/l_ce0).^b_act-1)/s_act).^a_act);
f_len_reg = exp(-((((l/l_ce0).^b_act-1)/s_act).^2+e^2).^(a_act/2));



a_pas = 2.38e-2;
b_pas = 5.31;
f_lce_pas = a_pas*exp(b_pas*(l/l_ce0-1));

figure
hold on
handle1 = title('F-L Relationship (active contractile element)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = xlabel('CE length (m)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = ylabel('Force amplification');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
% plot(l,f_lce_act,'r--', 'LineWidth',2)
plot(l,f_len_reg,'k-')
plot(l,f_lce_pas,'k-')
plot(l,f_len_reg+f_lce_pas,'k--')
axis([0,0.08,0,1.2])
hold off



Ksee = 315.4e3;
Q = 20;
fmax = 6000;

F_vec = linspace(0,6000,1e3);
L_vec = 1/Ksee*(F_vec + fmax/Q*log(1-0.9*exp(-Q/fmax*F_vec)) - fmax/Q*log(0.1));

figure(3)
hold on
handle1 = title('F-L Relationship (series-elastic element)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = xlabel('SEE length (m)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = ylabel('Force (N)');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
plot(L_vec,F_vec,'k', 'LineWidth',1)
% axis([0,0.08,0,1.2])
hold off


% Metabolic cost functions
% Note: velocities signs flipped (contraction = negative)

f_met_n = 0.23-0.16*exp(8*v/vmax);
f_met_p = 0.01-0.11*(-v/vmax)+0.06*exp(-23*v/vmax);
% f_met_p_relax = f_met_p.*0.5.*((v/vmax)+((v/vmax).^2+e^2)).^(1/2);
f_met_p_relax = f_met_p.*0.5.*((v/vmax)+((v/vmax).^2+e^2).^(1/2))./((v/vmax).^2+e^2).^(1/2);
f_met_n_relax = f_met_n.*0.5.*(-(v/vmax)+((v/vmax).^2+e^2).^(1/2))./((v/vmax).^2+e^2).^(1/2);

e_fit1 = 0.11; %0.15
off_fit1 = -0.08; %-0.05
f_met_n_fit1 = 0.22*0.5.*(-(v/vmax-off_fit1)+((v/vmax-off_fit1).^2+e_fit1^2).^(1/2))./((v/vmax-off_fit1).^2+e_fit1^2).^(1/2);
e_fit2 = 0.2; % 0.01;
f_met_n_fit2 = 0.01+(0.11*(v/vmax))*0.5.*((v/vmax)+((v/vmax).^2+e_fit2^2).^(1/2))./((v/vmax).^2+e_fit2^2).^(1/2);

figure
hold on
handle1 = title('F-V metabolic cost functions');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = xlabel('Velocity');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
handle1 = ylabel('Cost multiplier');
set(handle1,'Fontname','Arial')
set(handle1,'Fontweight','normal')
set(handle1,'Fontsize',10)
plot(v/vmax,f_met_p,'b--', 'LineWidth',2)
plot(v/vmax,f_met_n,'r--', 'LineWidth',2)
% plot(v/vmax,f_met_p_relax,'k-', 'LineWidth',1)
% plot(v/vmax,f_met_n_relax,'k-', 'LineWidth',1)
% plot(v/vmax,f_met_p_relax+f_met_n_relax,'k-', 'LineWidth',1)

plot(v/vmax,f_met_n_fit1,'c-', 'LineWidth',1)
plot(v/vmax,f_met_n_fit2,'c-', 'LineWidth',1)
plot(v/vmax,f_met_n_fit1+f_met_n_fit2,'k-', 'LineWidth',1)  %%%
% axis([-10,10,-1,1])
axis([-2,2,0,0.3])
hold off


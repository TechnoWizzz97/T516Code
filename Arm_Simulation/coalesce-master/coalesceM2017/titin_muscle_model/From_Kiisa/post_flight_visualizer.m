function [] = post_flight_visualizer(stim,data,params,varargin)

  figure()
  hold on
  plot(stim.relations.Active_Fl*params.Po,'linestyle','--');
  plot(stim.relations.Passive_Fl*params.Po,'linestyle',':')
  plot(data.act*params.Po,'linestyle','-.');
  title('FL Act and FCE');
  ylabel('N or %Act');
  xlabel('Data point');
  legend('FL Active','FL Passive','Activation','Fce','Fm');

  figure()
  subplot (5, 1, 1)
  hold on;
  plot (stim.forces.Fm,'linestyle',':');
  plot(data.force);
  title('WFM9 Series Spring and Measured');
  legend('Predicted','Measured');
  subplot(5, 1, 2);
  hold on;
  plot (stim.forces.Fce);
  plot(data.act*70,'linestyle',':');
  ylabel('N or Scaled Act');
  legend('Fce','70*Act');
  title('Fce Force');
  subplot(5, 1, 3);
  plot(stim.forces.Fcd);
  title('Fce Damper Force');
  subplot (5, 1, 4);
  plot (stim.forces.Fts);
  title('Titin Spring Force');
  subplot (5, 1, 5);
  plot (stim.forces.Ftd);
  title('Titin Damper Force');

  figure()
  subplot(6,1,1)
  plot(stim.positions.Thetap*57.2958);
  title('WFM9 Thetap');
  ylabel('Degrees');
  subplot(6,1,2);
  plot(stim.positions.Xp);
  title('Xp');
  ylabel('Displacement')
  subplot(6,1,3);
  plot(stim.positions.Xts);
  title('Xts');
  ylabel('Displacement')
  subplot(6,1,4);
  plot(stim.positions.Xss);
  title('Xss');
  ylabel('Displacement')
  subplot(6,1,5);
  plot(stim.positions.Xm(129:length(stim.positions.Xm)));

  title('Xm');
  ylabel('Displacement')
  subplot(6,1,6);
  plot(stim.positions.Xce);
  title('Xce');
  ylabel('Displacement')

  figure()
  subplot(2,1,1);
  plot(stim.accelerations.Xddotp);
  title('Xp double dot');
  subplot(2,1,2);
  plot(stim.accelerations.Thetaddotp);
  title('Theta double dot');

  figure()
  subplot(5,1,1);
  plot(stim.velocitys.Xdotp);
  title('Xp dot');
  subplot(5,1,2);
  plot(stim.velocitys.Thetadotp);
  title('Theta dot');
  subplot(5,1,3)
  plot(stim.velocitys.Xdotts);
  title('Xts dot');
  subplot(5,1,4)
  plot(stim.velocitys.Xdotce);
  title('Xce dot');
  subplot(5,1,5)
  plot(stim.velocitys.Xdotm);
  title('Xdotm');

end

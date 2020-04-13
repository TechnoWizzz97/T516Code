%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Scripting prototype by Christian Hubicki, Copyright 2015.
%
% optSand
%
% Framework code, COALESCE, by the inimitable and all-around smart guy, Mikhail Jones
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% clc
% clear('all');
% close all

% exportStatus

nNodes = 51;
nlp = DirectCollocation(nNodes);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Add Problem Parameters %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

optimFile = 'optRigid';

setParams


PLOT_ON = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Add Variable Structures %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add time variable
t = nlp.addTime;
T(1) = t;



% Initialize current phase index
iC = 0;


%%%%% Phase 1 - Stance (Partial Cone) %%%%%


iC = iC+1;
% Add states

% motor mass states
zm(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zm', 'Length', nNodes);
dzm(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzm', 'Length', nNodes);

% rod states
zr(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zr', 'Length', nNodes);
dzr(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzr', 'Length', nNodes);

% foot states
zf(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zf', 'Length', nNodes);
dzf(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzf', 'Length', nNodes);
ddzf(iC) = nlp.addVariable(0,-Inf,Inf,'Description','ddzf variable', 'Length', nNodes);

% Add motor force variables
fm(iC) = nlp.addVariable(1,-force_max,force_max,'Description','Motor force', 'Length', nNodes);

% Add reference trajectory acceleration
ddzd(iC) = nlp.addInput(0,-ref_accel_limit,ref_accel_limit,'Description','Reference trajectory', 'Length', nNodes);

% Add reference trajectory velocity
dzd(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzd', 'Length', nNodes);

% Add reference trajectory position
zd(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zd', 'Length', nNodes);

% Add velocity loop error
ev(iC) = nlp.addVariable(0,-Inf,Inf,'Description','Velocity loop error', 'Length', nNodes);
% Add integral of velocity loop error
iev(iC) = nlp.addState(1,-Inf,Inf,'Description','State: iev', 'Length', nNodes);


% Add jamming cone mass
mg(iC) = nlp.addState(0,-Inf,Inf,'Description','State: Jamming cone mass', 'Length', nNodes);
dmg_slack(iC) = nlp.addVariable(0,-Inf,Inf,'Description','dmg slack', 'Length', nNodes);



%%%%% Phase 3 - Flight Pre-Apex %%%%%


iC = iC+1;
% Add states

% motor mass states
zm(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zm', 'Length', nNodes);
dzm(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzm', 'Length', nNodes);

% rod states
zr(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zr', 'Length', nNodes);
dzr(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzr', 'Length', nNodes);

% foot states
zf(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zf', 'Length', nNodes);
dzf(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzf', 'Length', nNodes);
ddzf(iC) = nlp.addVariable(0,-Inf,Inf,'Description','ddzf variable', 'Length', nNodes);

% Add motor force variables
fm(iC) = nlp.addVariable(1,-force_max,force_max,'Description','Motor force', 'Length', nNodes);

% Add reference trajectory acceleration
ddzd(iC) = nlp.addInput(0,-ref_accel_limit,ref_accel_limit,'Description','Reference trajectory', 'Length', nNodes);

% Add reference trajectory velocity
dzd(iC) = nlp.addState(0,-Inf,Inf,'Description','State: dzd', 'Length', nNodes);

% Add reference trajectory position
zd(iC) = nlp.addState(0,-Inf,Inf,'Description','State: zd', 'Length', nNodes);

% Add velocity loop error
ev(iC) = nlp.addVariable(0,-Inf,Inf,'Description','Velocity loop error', 'Length', nNodes);
% Add integral of velocity loop error
iev(iC) = nlp.addState(1,-Inf,Inf,'Description','State: iev', 'Length', nNodes);


% Add jamming cone mass
mg(iC) = nlp.addState(0,-Inf,Inf,'Description','State: Jamming cone mass', 'Length', nNodes);
% dmg_slack(iC) = nlp.addVariable(0,-Inf,Inf,'Description','dmg slack', 'Length', nNodes);




t.initialGuess = .1;
t = nlp.addTime;
t.initialGuess = .1;
T(2) = T(1)+t;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Add Dynamics %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


z_cone_start = zf(1).initial;

iC = 0;


% 1: STANCE (PARTIAL CONE)
iC = iC+1;
SpringForce
% GrainForce

% Add dynamics
nlp.addPdeConstraint(zm(iC), dzm(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zm dynamics');
nlp.addPdeConstraint(dzm(iC), fm(iC)/ma - g, T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzm dynamics');

nlp.addPdeConstraint(zr(iC), dzr(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zr dynamics');
nlp.addPdeConstraint(dzr(iC), -fm(iC)/mr - g + fSpring/mr, T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzr dynamics');

nlp.addPdeConstraint(zf(iC), dzf(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zf dynamics');
nlp.addPdeConstraint(dzf(iC), ddzf(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzf dynamics');
nlp.addConstraint(0,ddzf(iC),0)


nlp.addPdeConstraint(dzd(iC), ddzd(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzd dynamics');
nlp.addPdeConstraint(zd(iC), dzd(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zd dynamics');

nlp.addPdeConstraint(iev(iC), ev(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zd dynamics');

% nlp.addPdeConstraint(mg(iC), dmg_slack(iC), T(iC), ...
% 	'Method', 'Trapezoidal', ...
% 	'Description', 'mg dynamics');
% nlp.addConstraint(0,dmg_slack(iC)-dmg,0)






% 2: FLIGHT
iC = iC+1;
SpringForce

% Add dynamics
nlp.addPdeConstraint(zm(iC), dzm(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zm dynamics');
nlp.addPdeConstraint(dzm(iC), fm(iC)/ma - g, T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzm dynamics');

nlp.addPdeConstraint(zr(iC), dzr(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zr dynamics');
nlp.addPdeConstraint(dzr(iC), -fm(iC)/mr - g + fSpring/mr, T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzr dynamics');

nlp.addPdeConstraint(zf(iC), dzf(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zf dynamics');
nlp.addPdeConstraint(dzf(iC), ddzf(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzf dynamics');
nlp.addConstraint(0,ddzf(iC)-(-g - fSpring/mf),0)


nlp.addPdeConstraint(dzd(iC), ddzd(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'dzd dynamics');
nlp.addPdeConstraint(zd(iC), dzd(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zd dynamics');

nlp.addPdeConstraint(iev(iC), ev(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'zd dynamics');

nlp.addPdeConstraint(mg(iC), 0*zf(iC), T(iC), ...
	'Method', 'Trapezoidal', ...
	'Description', 'mg dynamics');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Add Constraints %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% nlp.addConstraint(1,t,1)
% nlp.addConstraint(0,ddla,0)

% Minimum phase times
nlp.addConstraint(0.01,T(1),inf)
nlp.addConstraint(0.01,T(2),inf)


i0 = 1;
iF = 2;

% Phase indices
iC = 1;
iCn = iC+1;

SpringForce

% Initial Condition
nlp.addConstraint(0,zf(iC).initial,0)
% nlp.addConstraint(0,fGranularInit - (ma+mr+mf)*g,0)
nlp.addConstraint(0,fSpringInit - m*g,0)
nlp.addConstraint(0,dzm(iC).initial,0)
nlp.addConstraint(0,dzr(iC).initial,0)
nlp.addConstraint(0,dzf(iC).initial,0)

% nlp.addConstraint(0,mg(iC).initial,0)
% nlp.addConstraint(0,dmgInit,0)

% initial position error is zero
nlp.addConstraint(0,(zm(iC).initial-zr(iC).initial)-zd(iC).initial,0)

% Start with zero reference velocity
nlp.addConstraint(0,dzd(iC).initial,0)

% Takeoff force condition
nlp.addConstraint(0,fSpringFinal+mf*g,0)
% nlp.addConstraint(0,dzf(iC).final,0)
% Foot must have non-negative acceleration at takeoff
% nlp.addConstraint(-Inf,fSpringFinal+mf*g,0)

% Start with zero integral error
nlp.addConstraint(0,iev(iC).initial,0)

% Positive GRF
nlp.addConstraint(0,fSpring+mf*g,inf)

% Fully developed cone condition
% nlp.addConstraint(-Inf,mu*(z_cone_start-zf(iC).final)-R*tan(theta),0)

% Rod above foot
nlp.addConstraint(rod_length/2+spring_clearance,zr(iC)-zf(iC),inf)

% Motor-rod displacement limit
nlp.addConstraint(-rod_length/2,zm(iC)-zr(iC),rod_length/2)

% Terrain has no restorative motion
% nlp.addConstraint(-Inf,dzf(iC),0)



% Compute Velocity Error
nlp.addConstraint(0,Pp*(zd(iC)-(zm(iC)-zr(iC)))+Vff*dzd(iC)-(dzm(iC)-dzr(iC)) - ev(iC),0)
% Feedback control law
nlp.addConstraint(0,Km*(Vp*ev(iC) + Vi*iev(iC)) - fm(iC),0)



% Continuity guard
% positions
nlp.addConstraint(0,zm(iCn).initial - zm(iC).final,0)
nlp.addConstraint(0,zr(iCn).initial - zr(iC).final,0)
nlp.addConstraint(0,zf(iCn).initial - zf(iC).final,0)
% velocities
nlp.addConstraint(0,dzm(iCn).initial - dzm(iC).final,0)
nlp.addConstraint(0,dzr(iCn).initial - dzr(iC).final,0)
nlp.addConstraint(0,dzf(iCn).initial - dzf(iC).final,0)

% Reference trajectories
nlp.addConstraint(0,zd(iCn).initial - zd(iC).final,0)
nlp.addConstraint(0,dzd(iCn).initial - dzd(iC).final,0)
% Integral error term
nlp.addConstraint(0,iev(iCn).initial - iev(iC).final,0)

nlp.addConstraint(0,mg(iCn).initial - mg(iC).final,0)

% Update phase indices
iC = 2;
iCn = iC+1;
SpringForce


% No added mass dynamics
% nlp.addConstraint(0,dmgInit,0)

% Rod above foot
nlp.addConstraint(rod_length/2+spring_clearance,zr(iC)-zf(iC),inf)
% Motor-rod displacement limit
nlp.addConstraint(-rod_length/2,zm(iC)-zr(iC),rod_length/2)

% Compute Velocity Error
nlp.addConstraint(0,Pp*(zd(iC)-(zm(iC)-zr(iC)))+Vff*dzd(iC)-(dzm(iC)-dzr(iC)) - ev(iC),0)
% Feedback control law
nlp.addConstraint(0,Km*(Vp*ev(iC) + Vi*iev(iC)) - fm(iC),0)

% Apex Condition
% nlp.addConstraint(apex_height,(ma*zm(iC).final + mr*zr(iC).final + mf*zf(iC).final)/(ma+mr+mf),apex_height)
nlp.addConstraint(0,(ma*dzm(iC).final + mr*dzr(iC).final + mf*dzf(iC).final)/(ma+mr+mf),0)
nlp.addConstraint(rod_hop_height,(zr(iF).final - zr(i0).initial),rod_hop_height)

% ADDITIONAL TASK CONSTRAINTS
% Return motor to initial position
nlp.addConstraint(0,(zm(i0).initial-zr(i0).initial) - (zm(iF).final-zr(iF).final),0)
% Bring motor to a halt and its command
nlp.addConstraint(0,dzm(iF).final-dzr(iF).final,0)
nlp.addConstraint(0,dzd(iF).final,0)
nlp.addConstraint(0,fm(iF).final,0)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Add Objective %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% nlp.addObjective(trapz(fm(1)'*fm(1), T(1))+trapz(fm(2)'*fm(2), T(2)), ...
% 	'Description','Minimize force squared')

% Minimize actuator velocity
% nlp.addObjective( ...
% 	trapz((dzm(1)-dzr(1))'*(dzm(1)-dzr(1)), T(1))+ ...
% 	trapz((dzm(2)-dzr(2))'*(dzm(2)-dzr(2)), T(2))+ ...
% 	trapz((dzm(3)-dzr(3))'*(dzm(3)-dzr(3)), T(3))+ ...
% 	trapz((dzm(4)-dzr(4))'*(dzm(4)-dzr(4)), T(4)), ...
% 	'Description','Minimize actuator velocity')

% Minimize actuator velocity and force
% nlp.addObjective( ...
% 	trapz((dzm(1)-dzr(1))'*(dzm(1)-dzr(1)), T(1))+ ...
% 	trapz((dzm(2)-dzr(2))'*(dzm(2)-dzr(2)), T(2))+ ...
% 	trapz((dzm(3)-dzr(3))'*(dzm(3)-dzr(3)), T(3))+ ...
% 	trapz((dzm(4)-dzr(4))'*(dzm(4)-dzr(4)), T(4))+ ...
% 	(trapz(fm(1)'*fm(1), T(1))+ ...
% 	trapz(fm(2)'*fm(2), T(2))+ ...
% 	trapz(fm(3)'*fm(3), T(3))+ ...
% 	trapz(fm(4)'*fm(4), T(4)))/Km^2*Rc, ...
% 	'Description','Minimize actuator velocity')

nlp.addObjective( ...
	trapz((dzm(1)-dzr(1))'*(dzm(1)-dzr(1)), T(1))+ ...
	trapz((dzm(2)-dzr(2))'*(dzm(2)-dzr(2)), T(2))+ ...
	(trapz(fm(1)'*fm(1), T(1))+ ...
	trapz(fm(2)'*fm(2), T(2)))*force_weight + ...
	(trapz(ddzd(1)'*ddzd(1), T(1))+ ...
	trapz(ddzd(2)'*ddzd(2), T(2)))*accel_weight, ...
	'Description','Minimize actuator velocity')




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% SOLVE %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Construct optimizer interface object, export and solve
optim = Ipopt(nlp);
optim.export;
optim.options.ipopt.max_iter = 1.5e3;

tic
optim.solve;
optim_run_time = toc










             %%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%% Post-processing %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
             %%%%%%%%%%%%%%%%%%%



zm_sol = squeeze(eval(zm));
zr_sol = squeeze(eval(zr));
zf_sol = squeeze(eval(zf));
fm_sol = squeeze(eval(fm));

dzm_sol = squeeze(eval(dzm));
dzr_sol = squeeze(eval(dzr));
dzf_sol = squeeze(eval(dzf));

zd_sol = squeeze(eval(zd));
dzd_sol = squeeze(eval(dzd));

mg_sol = squeeze(eval(mg));

t_sol1 = linspace(0,squeeze(eval(T(1))),nNodes);
t_sol2 = linspace(0,squeeze(eval(T(2))),nNodes)+t_sol1(end);


durations = T.eval;

time_vector = [];
zm_vector = [];
zr_vector = [];
zf_vector = [];
dzm_vector = [];
dzr_vector = [];
dzf_vector = [];

mg_vector = [];

zd_vector = [];
dzd_vector = [];

for iter_loop = 1:numel(T)
	time_vector = [time_vector, linspace(sum(durations(1:iter_loop-1)), sum(durations(1:iter_loop)), nNodes)];

	zm_vector = [zm_vector, zm_sol(iter_loop,:)];
	zr_vector = [zr_vector, zr_sol(iter_loop,:)];
	zf_vector = [zf_vector, zf_sol(iter_loop,:)];
	dzm_vector = [dzm_vector, dzm_sol(iter_loop,:)];
	dzr_vector = [dzr_vector, dzr_sol(iter_loop,:)];
	dzf_vector = [dzf_vector, dzf_sol(iter_loop,:)];

	mg_vector = [mg_vector, mg_sol(iter_loop,:)];

	zd_vector = [zd_vector, zd_sol(iter_loop,:)];
	dzd_vector = [dzd_vector, dzd_sol(iter_loop,:)];
end

time_vector = time_vector + eps.*cumsum([1:numel(time_vector)]);

response = struct;
response.time = {time_vector};
response.zm = zm_vector;
response.zr = zr_vector;
response.zf = zf_vector;

if(PLOT_ON)
subplot(4,1,1)
hold on
plot(t_sol1, zm_sol(1,:), 'r-', t_sol1, zr_sol(1,:), 'b-', t_sol1, zf_sol(1,:), 'm-')
plot(t_sol2(1), zm_sol(2,1), 'r.', t_sol2(1), zr_sol(2,1), 'b.', t_sol2(1), zf_sol(2,1), 'm.')
plot(t_sol2, zm_sol(2,:), 'r-', t_sol2, zr_sol(2,:), 'b-', t_sol2, zf_sol(2,:), 'm-')
ylabel('Height (m)')
legend('mass','rod','foot')
hold off
subplot(4,1,2)
hold on
plot(t_sol1, fm_sol(1,:), 'g-')
plot(t_sol2(1), fm_sol(2,1), 'g.')
plot(t_sol2, fm_sol(2,:), 'g-')
ylabel('Force (N)')
legend('actuator force')
hold off
subplot(4,1,3)
hold on
plot(t_sol1, (ma*zm_sol(1,:)+mr*zr_sol(1,:)+mf*zf_sol(1,:))/(ma+mr+mf), 'k-')
plot(t_sol2(1), (ma*zm_sol(2,1)+mr*zr_sol(2,1)+mf*zf_sol(2,1))/(ma+mr+mf), 'k.')
plot(t_sol2, (ma*zm_sol(2,:)+mr*zr_sol(2,:)+mf*zf_sol(2,:))/(ma+mr+mf), 'k-')
xlabel('Time (s)')
ylabel('Height (m)')
legend('center-of-mass trajectory')
hold off
end

if(PLOT_ON)
scene = HopperScene(response);
Player(scene);
end

dt_interp = 0.001;
t_interp = [0:dt_interp:max(time_vector)];
zm_interp = interp1(time_vector, zm_vector, t_interp);
zr_interp = interp1(time_vector, zr_vector, t_interp);
dzm_interp = interp1(time_vector, dzm_vector, t_interp);
dzr_interp = interp1(time_vector, dzr_vector, t_interp);
zd_interp = interp1(time_vector, zd_vector, t_interp);
dzd_interp = interp1(time_vector, dzd_vector, t_interp);

zm_interp(1) = zm_vector(1);
zr_interp(1) = zr_vector(1);
zd_interp(1) = zd_vector(1);
dzm_interp(1) = dzm_vector(1);
dzr_interp(1) = dzr_vector(1);

% csvwrite(['data.csv_',datetime('now')], [t_interp.', (zm_interp - zr_interp).']);
csvwrite('simcommanddata.csv', [t_interp.', (zm_interp - zr_interp).']);

if(PLOT_ON)
comp_fig = figure
hold on
zr_init = zr_sol(1,1);
motor_offset = 0; % 0.025
plot(t_interp, zm_interp - zr_interp + motor_offset, 'k-')
plot(t_interp, zd_interp + motor_offset, 'k--')
plot(t_sol1, zr_sol(1,:)-zr_init, 'b-')
plot(t_sol2(1), zr_sol(2,1)-zr_init, 'b.')
plot(t_sol2, zr_sol(2,:)-zr_init, 'b-')
hold off
end



zm_offset = 0.025; %0.025;
zr_offset = 0;

% zd_offset = 0.02;
zd_offset = -min(zd_interp)+0.005;

zm_adjust = zm_interp + zm_offset;
zr_adjust = zr_interp + zr_offset;

% motor_pos_command = zm_adjust - zr_adjust;
% motor_vel_command = dzm_interp - dzr_interp;

motor_pos_command = zd_interp+zd_offset;
motor_vel_command = dzd_interp;

% dividing mm by 0.00625
encoder_conversion = 1000/0.00625;
motor_pos_encoder = motor_pos_command*encoder_conversion;
motor_vel_encoder = motor_vel_command*encoder_conversion;
time_coding = 1*ones(size(motor_pos_encoder));
time_coding(end) = 0;
time_debug = time_vector;

robot_directory_name = 'exports';
directory_name = 'ExperimentalOutput';
right_now = datetime('now');
time_string = datestr(right_now,'yyyymmddTHHMMSS');
id_string = [optimFile, time_string];
% 
% storeParams
% 
% if(EXPORT_ON)
% 	dlmwrite([robot_directory_name filesep 'updated_trajectory.txt'], [motor_pos_encoder.', motor_vel_encoder.', time_coding.'] , ' ')
% 	dlmwrite([directory_name filesep  datestr(right_now,'yyyymmddTHHMMSS'), '_robot', '.txt'], [motor_pos_encoder.', motor_vel_encoder.', time_coding.'] , ' ')
% 
% 
% 	fileID = fopen([directory_name filesep, datestr(right_now,'yyyymmddTHHMMSS'), 'param_', '.txt'],'w');
% 	for iter = 1:numel(param_names)
% 		fprintf(fileID,[param_names{iter}, ': ']);
% 		fprintf(fileID,[num2str(param_values{iter}), '\n']);
% 	end
% 	fclose(fileID);
% 
% 	fileID = fopen([robot_directory_name filesep 'updated_params.txt'],'w');
% 	for iter = 1:numel(param_names)
% 		fprintf(fileID,[param_names{iter}, ': ']);
% 		fprintf(fileID,[num2str(param_values{iter}), ' \n']);
% 	end
% 	fclose(fileID);
% 
% 	save([directory_name filesep 'WS_' optimFile time_string '.mat']) 
% 	
% 	savefig(comp_fig,[directory_name filesep 'FIG_' optimFile time_string '.fig'])
% 
% end
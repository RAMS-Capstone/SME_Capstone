function build_gen3_pmsm
% Creates new models only. Original SLX files are loaded read-only.
src='PMSM_1DOF_position_compare_v1'; load_system(src);
g=@(b) str2num(get_param([src '/' b],'Gain')); %#ok<ST2NM>
p.Rs=g('PMSM_manual/Rs'); p.Ld=1/g('PMSM_manual/inv_Ld');
p.Lq=1/g('PMSM_manual/inv_Lq'); p.pp=g('PMSM_manual/pole_pairs');
p.psi=str2double(get_param([src '/PMSM_manual/psi_f'],'Value'));
p.Kt=g('PMSM_manual/PM_torque');
p.N=1/g('Motor_Mechanics/load_to_motor');
p.Jtotal_motor=1/g('Motor_Mechanics/inv_Jtotal');
% The source expression is explicitly 0.01 + 0.01/2^2.
assert(strcmp(get_param([src '/Motor_Mechanics/inv_Jtotal'],'Gain'),'1/(0.01+0.01/2^2)'));
p.Jm=0.01; p.Jload_test=0.01; p.Bm=g('Motor_Mechanics/Bm');
p.Kpd=str2double(get_param([src '/PI_d'],'P'));
p.Kpq=str2double(get_param([src '/PI_q'],'P'));
p.Kid=str2double(get_param([src '/PI_d'],'I'));
p.Kiq=str2double(get_param([src '/PI_q'],'I'));
p.Imax=str2double(get_param([src '/iq_ref_limit'],'UpperLimit'));
p.Vqmax=str2double(get_param([src '/vq_limit'],'UpperLimit'));
p.s=1; % Coordinate convention, NOT verified hardware polarity.
p.Kaw=p.Kiq/p.Kpq; % New PI back-calculation, inverse PI integral time.
assert(abs(p.Kt-1.5*p.pp*p.psi)<1e-12);
assert(abs(p.Jtotal_motor-p.Jm-p.Jload_test/p.N^2)<1e-12);
save('pmsm_validation_parameters.mat','p');
m='Gen3_J2_PMSM_Actuator'; fresh(m);
w=get_param(m,'ModelWorkspace'); w.assignin('p',p);
set_param(m,'Solver','ode23t','MaxStep','1e-4','RelTol','1e-6','AbsTol','1e-8','StopTime','1');
set_param(m,'Description','Validation dq torque actuator. No position/speed loop or mechanical inertia. Ideal rigid lossless gear. q voltage limit inherited, not a complete inverter.');
blk(m,'In1','tau_cmd_J','Port','1','PortDimensions','1','Unit','N*m','Interpolate','off');
blk(m,'In1','omega_J','Port','2','PortDimensions','1','Unit','rad/s','Interpolate','off');
blk(m,'Subsystem','PMSM_Actuator'); s=[m '/PMSM_Actuator']; Simulink.SubSystem.deleteContents(s);
blk(s,'In1','tau_cmd_J','Port','1','PortDimensions','1','Unit','N*m');
blk(s,'In1','omega_J','Port','2','PortDimensions','1','Unit','rad/s');
blk(s,'Gain','Torque_to_iq','Gain','1/(p.s*p.N*p.Kt)');
blk(s,'Saturation','Current_command_limit','UpperLimit','p.Imax','LowerLimit','-p.Imax');
blk(s,'Gain','Joint_to_motor_speed','Gain','p.s*p.N');
blk(s,'Gain','Electrical_speed','Gain','p.pp');
add_block([src '/PMSM_manual'],[s '/PMSM_dq']);
% Replace literals by the extracted, model-local parameters.
for pair={{'Rs','p.Rs'},{'Rs_q','p.Rs'},{'inv_Ld','1/p.Ld'},{'inv_Lq','1/p.Lq'}, ...
 {'Ld_id','p.Ld'},{'Ld_iq','p.Lq'},{'pole_pairs','p.pp'}, ...
 {'PM_torque','p.Kt'},{'Reluctance_torque','1.5*p.pp*(p.Ld-p.Lq)'}}
 v=pair{1}; set_param([s '/PMSM_dq/' v{1}],'Gain',v{2});
end
set_param([s '/PMSM_dq/psi_f'],'Value','p.psi');
blk(s,'Constant','id_ref','Value','0');
blk(s,'Sum','id_error','Inputs','+-'); blk(s,'Sum','iq_error','Inputs','+-');
for ax={'d','q'}
 a=ax{1}; blk(s,'Gain',['Kp_' a],'Gain',['p.Kp' a]);
 blk(s,'Gain',['Ki_' a],'Gain',['p.Ki' a]);
 blk(s,'Integrator',['PI_state_' a]);
 blk(s,'Sum',['PI_' a],'Inputs','++');
 wire(s,['i' a '_error/1'],['Kp_' a '/1']); wire(s,['i' a '_error/1'],['Ki_' a '/1']);
 wire(s,['Kp_' a '/1'],['PI_' a '/1']); wire(s,['PI_state_' a '/1'],['PI_' a '/2']);
end
blk(s,'Sum','q_integrator_drive','Inputs','++');
blk(s,'Sum','q_voltage_residual','Inputs','+-');
blk(s,'Gain','Anti_windup_q','Gain','p.Kaw');
blk(s,'Gain','Lq_iq','Gain','p.Lq'); blk(s,'Gain','Ld_id','Gain','p.Ld');
blk(s,'Constant','PM_flux','Value','p.psi'); blk(s,'Sum','d_flux','Inputs','++');
blk(s,'Product','d_decoupling'); blk(s,'Product','q_decoupling');
blk(s,'Sum','vd','Inputs','+-'); blk(s,'Sum','vq_raw','Inputs','++');
blk(s,'Saturation','q_voltage_limit','UpperLimit','p.Vqmax','LowerLimit','-p.Vqmax');
blk(s,'Gain','Motor_to_joint_torque','Gain','p.s*p.N');
blk(s,'Out1','tau_act_J','Port','1','PortDimensions','1','Unit','N*m');
blk(s,'Mux','Diagnostics','Inputs','9'); blk(s,'Out1','diag','Port','2');
edges={ 'tau_cmd_J/1','Torque_to_iq/1';'Torque_to_iq/1','Current_command_limit/1'; ...
 'omega_J/1','Joint_to_motor_speed/1';'Joint_to_motor_speed/1','Electrical_speed/1'; ...
 'Joint_to_motor_speed/1','PMSM_dq/3';'id_ref/1','id_error/1';'PMSM_dq/1','id_error/2'; ...
 'Current_command_limit/1','iq_error/1';'PMSM_dq/2','iq_error/2'; ...
 'Ki_d/1','PI_state_d/1';'Ki_q/1','q_integrator_drive/1'; ...
 'q_integrator_drive/1','PI_state_q/1';'q_voltage_limit/1','q_voltage_residual/1'; ...
 'vq_raw/1','q_voltage_residual/2';'q_voltage_residual/1','Anti_windup_q/1'; ...
 'Anti_windup_q/1','q_integrator_drive/2';'PMSM_dq/2','Lq_iq/1';'PMSM_dq/1','Ld_id/1'; ...
 'Ld_id/1','d_flux/1';'PM_flux/1','d_flux/2';'Lq_iq/1','d_decoupling/1'; ...
 'Electrical_speed/1','d_decoupling/2';'d_flux/1','q_decoupling/1'; ...
 'Electrical_speed/1','q_decoupling/2';'PI_d/1','vd/1';'d_decoupling/1','vd/2'; ...
 'PI_q/1','vq_raw/1';'q_decoupling/1','vq_raw/2';'vq_raw/1','q_voltage_limit/1'; ...
 'vd/1','PMSM_dq/1';'q_voltage_limit/1','PMSM_dq/2'; ...
 'PMSM_dq/3','Motor_to_joint_torque/1';'Motor_to_joint_torque/1','tau_act_J/1'};
for k=1:size(edges,1),wire(s,edges{k,1},edges{k,2});end
signals={'PMSM_dq/1','PMSM_dq/2','Current_command_limit/1','PMSM_dq/3', ...
 'Joint_to_motor_speed/1','vd/1','q_voltage_limit/1','vq_raw/1','PI_state_q/1'};
for k=1:numel(signals),wire(s,signals{k},sprintf('Diagnostics/%d',k));end
wire(s,'Diagnostics/1','diag/1');
blk(m,'Out1','tau_act_J','Port','1','Unit','N*m'); blk(m,'Out1','diag','Port','2');
blk(m,'ToWorkspace','diagnostic_log','VariableName','diagnostic_log','SaveFormat','Timeseries');
blk(m,'ToWorkspace','torque_log','VariableName','torque_log','SaveFormat','Timeseries');
wire(m,'tau_cmd_J/1','PMSM_Actuator/1');wire(m,'omega_J/1','PMSM_Actuator/2');
wire(m,'PMSM_Actuator/1','tau_act_J/1');wire(m,'PMSM_Actuator/2','diag/1');
wire(m,'PMSM_Actuator/1','torque_log/1');wire(m,'PMSM_Actuator/2','diagnostic_log/1');
Simulink.BlockDiagram.arrangeSystem(s); Simulink.BlockDiagram.arrangeSystem(m);
set_param(m,'SimulationCommand','update'); save_system(m);
% Separate mechanics fixture: one inertia only, expressed on joint side.
h='Gen3_J2_PMSM_Standalone'; fresh(h); hw=get_param(h,'ModelWorkspace'); hw.assignin('p',p);
set_param(h,'Solver','ode23t','MaxStep','1e-4','RelTol','1e-6','AbsTol','1e-8','StopTime','1');
add_block([m '/PMSM_Actuator'],[h '/PMSM_Actuator']);
blk(h,'Step','Torque_command','Time','0.05','Before','0','After','0.2');
blk(h,'Step','Load_torque','Time','0.5','Before','0','After','0.1');
blk(h,'Sum','Net_joint_torque','Inputs','+--');
blk(h,'Gain','Test_inertia_only','Gain','1/(p.Jload_test+p.N^2*p.Jm)');
blk(h,'Gain','Reflected_damping','Gain','p.N^2*p.Bm');
blk(h,'Integrator','Joint_speed'); blk(h,'Integrator','Joint_position');
blk(h,'ToWorkspace','speed_log','VariableName','speed_log','SaveFormat','Timeseries');
blk(h,'ToWorkspace','position_log','VariableName','position_log','SaveFormat','Timeseries');
blk(h,'ToWorkspace','torque_log','VariableName','torque_log','SaveFormat','Timeseries');
blk(h,'ToWorkspace','diagnostic_log','VariableName','diagnostic_log','SaveFormat','Timeseries');
edges={'Torque_command/1','PMSM_Actuator/1';'Joint_speed/1','PMSM_Actuator/2'; ...
 'PMSM_Actuator/1','Net_joint_torque/1';'Load_torque/1','Net_joint_torque/2'; ...
 'Joint_speed/1','Reflected_damping/1';'Reflected_damping/1','Net_joint_torque/3'; ...
 'Net_joint_torque/1','Test_inertia_only/1';'Test_inertia_only/1','Joint_speed/1'; ...
 'Joint_speed/1','Joint_position/1';'Joint_speed/1','speed_log/1'; ...
 'Joint_position/1','position_log/1';'PMSM_Actuator/1','torque_log/1'; ...
 'PMSM_Actuator/2','diagnostic_log/1'};
for k=1:size(edges,1),wire(h,edges{k,1},edges{k,2});end
Simulink.BlockDiagram.arrangeSystem(h);set_param(h,'SimulationCommand','update');save_system(h);
close_system(src,0); close_system(m,0); close_system(h,0);
fprintf('Created actuator and independent mechanical fixture. Original models not saved.\n');
end
function fresh(m)
if bdIsLoaded(m),close_system(m,0);end
new_system(m);
end
function blk(s,type,name,varargin)
switch type
 case {'In1','Constant','Step'}, folder='Sources';
 case {'Out1','ToWorkspace'}, folder='Sinks';
 case 'Integrator', folder='Continuous';
 case 'Saturation', folder='Discontinuities';
 case 'Mux', folder='Signal Routing';
 case 'Subsystem', folder='Ports & Subsystems';
 otherwise, folder='Math Operations';
end
if strcmp(type,'ToWorkspace'),type='To Workspace';end
add_block(['simulink/' folder '/' type],[s '/' name],varargin{:});
end
function wire(s,a,b)
add_line(s,a,b,'autorouting','on');
end

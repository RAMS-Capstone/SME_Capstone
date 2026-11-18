function build_stage1_pmsm
% BUILD_STAGE1_PMSM  Creates J2_PMSM_Stage1.slx: hand-built dq PMSM + FOC current loop.
%   Every equation is its own block (Gain / Sum / Product / Integrator) so it can be read
%   against the equations in the stage-1 doc. No library motor block is used.
%
%   Subsystems
%     PMSM_dq_manual  : did/dt = (vd - Rs*id + we*Lq*iq)/Ld
%                       diq/dt = (vq - Rs*iq - we*Ld*id - we*psi)/Lq
%                       Te     = 1.5*pp*(psi*iq + (Ld-Lq)*id*iq)
%     FOC_current_ctrl: PI (Kp, Ki) + decoupling feed-forward + |v| <= Vmax (d priority)
%                       + back-calculation anti-windup, iq command limit +-Imax
%   Root harness
%     In1 iq_cmd [A], In2 wm_imposed [rad/s]; free_rotor = 0 -> imposed speed,
%     free_rotor = 1 -> J_test*dwm/dt = Te. Log 'stage1_log' (13 columns, see LOG_COLUMNS).
m = 'J2_PMSM_Stage1';
p = j2_params();
if bdIsLoaded(m), close_system(m,0); end
new_system(m);
w = get_param(m,'ModelWorkspace');
w.assignin('p',p); w.assignin('free_rotor',0);
set_param(m,'Solver','ode23t','MaxStep','1e-5','RelTol','1e-6','AbsTol','1e-8','StopTime','0.02');
set_param(m,'Description','Stage 1: hand-built dq PMSM (maxon EC-i 52 24V) + FOC current loop. Average-value inverter (no PWM).');

%% ---------------- PMSM_dq_manual ----------------
s = [m '/PMSM_dq_manual'];
add_block('simulink/Ports & Subsystems/Subsystem', s); Simulink.SubSystem.deleteContents(s);
blk(s,'In1','vd','Port','1'); blk(s,'In1','vq','Port','2'); blk(s,'In1','wm','Port','3');
blk(s,'Gain','we','Gain','p.pp');                                   % electrical speed
% d axis
blk(s,'Sum','vd_balance','Inputs','+-+');                             % vd - Rs*id + we*Lq*iq
blk(s,'Gain','Rs_d','Gain','p.Rs');
blk(s,'Gain','Lq_iq','Gain','p.Lq'); blk(s,'Product','we_Lq_iq');
blk(s,'Gain','inv_Ld','Gain','1/p.Ld'); blk(s,'Integrator','id_state','InitialCondition','0');
% q axis
blk(s,'Sum','vq_balance','Inputs','+---');                            % vq - Rs*iq - we*Ld*id - we*psi
blk(s,'Gain','Rs_q','Gain','p.Rs');
blk(s,'Gain','Ld_id','Gain','p.Ld'); blk(s,'Product','we_Ld_id');
blk(s,'Gain','back_emf','Gain','p.psi');                             % we*psi
blk(s,'Gain','inv_Lq','Gain','1/p.Lq'); blk(s,'Integrator','iq_state','InitialCondition','0');
% torque
blk(s,'Gain','PM_torque','Gain','1.5*p.pp*p.psi');
blk(s,'Product','id_x_iq'); blk(s,'Gain','Reluctance_torque','Gain','1.5*p.pp*(p.Ld-p.Lq)');
blk(s,'Sum','Te_sum','Inputs','++');
blk(s,'Out1','id','Port','1'); blk(s,'Out1','iq','Port','2'); blk(s,'Out1','Te','Port','3');
E = { 'wm/1','we/1';
      'vd/1','vd_balance/1'; 'id_state/1','Rs_d/1'; 'Rs_d/1','vd_balance/2';
      'iq_state/1','Lq_iq/1'; 'we/1','we_Lq_iq/1'; 'Lq_iq/1','we_Lq_iq/2'; 'we_Lq_iq/1','vd_balance/3';
      'vd_balance/1','inv_Ld/1'; 'inv_Ld/1','id_state/1';
      'vq/1','vq_balance/1'; 'iq_state/1','Rs_q/1'; 'Rs_q/1','vq_balance/2';
      'id_state/1','Ld_id/1'; 'we/1','we_Ld_id/1'; 'Ld_id/1','we_Ld_id/2'; 'we_Ld_id/1','vq_balance/3';
      'we/1','back_emf/1'; 'back_emf/1','vq_balance/4';
      'vq_balance/1','inv_Lq/1'; 'inv_Lq/1','iq_state/1';
      'iq_state/1','PM_torque/1'; 'id_state/1','id_x_iq/1'; 'iq_state/1','id_x_iq/2';
      'id_x_iq/1','Reluctance_torque/1'; 'PM_torque/1','Te_sum/1'; 'Reluctance_torque/1','Te_sum/2';
      'id_state/1','id/1'; 'iq_state/1','iq/1'; 'Te_sum/1','Te/1'};
wireall(s,E); Simulink.BlockDiagram.arrangeSystem(s);

%% ---------------- FOC_current_ctrl ----------------
c = [m '/FOC_current_ctrl'];
add_block('simulink/Ports & Subsystems/Subsystem', c); Simulink.SubSystem.deleteContents(c);
blk(c,'In1','iq_cmd','Port','1'); blk(c,'In1','id','Port','2');
blk(c,'In1','iq','Port','3');     blk(c,'In1','wm','Port','4');
blk(c,'Saturation','iq_cmd_limit','UpperLimit','p.Imax','LowerLimit','-p.Imax');
blk(c,'Constant','id_ref','Value','0');
blk(c,'Gain','we','Gain','p.pp');
for ax = {'d','q'}
    a = ax{1};
    blk(c,'Sum',['e' a],'Inputs','+-');
    blk(c,'Gain',['Kp_' a],'Gain','p.Kp'); blk(c,'Gain',['Ki_' a],'Gain','p.Ki');
    blk(c,'Sum',['x' a '_drive'],'Inputs','++');                      % Ki*e + Kaw*(v - v_raw)
    blk(c,'Integrator',['x' a],'InitialCondition','0');
    blk(c,'Sum',['PI_' a],'Inputs','++');
    blk(c,'Sum',['v' a '_excess'],'Inputs','+-');                     % v_applied - v_raw (<= 0 when saturated)
    blk(c,'Gain',['Kaw_' a],'Gain','p.Kaw');
end
% decoupling feed-forward: vd_ff = -we*Lq*iq,  vq_ff = we*(Ld*id + psi)
blk(c,'Gain','Lq_iq','Gain','p.Lq'); blk(c,'Product','we_Lq_iq');
blk(c,'Gain','Ld_id','Gain','p.Ld'); blk(c,'Constant','psi','Value','p.psi');
blk(c,'Sum','flux_d','Inputs','++'); blk(c,'Product','we_flux_d');
blk(c,'Sum','vd_raw','Inputs','+-'); blk(c,'Sum','vq_raw','Inputs','++');
% voltage vector limit, d priority: |vd|<=Vmax, |vq|<=sqrt(Vmax^2-vd^2)
blk(c,'Saturation','vd_limit','UpperLimit','p.Vmax','LowerLimit','-p.Vmax');
blk(c,'Product','vd_sq'); blk(c,'Constant','Vmax_sq','Value','p.Vmax^2');
blk(c,'Sum','vq_room_sq','Inputs','+-');
blk(c,'Saturation','nonneg','UpperLimit','inf','LowerLimit','0');
blk(c,'Sqrt','vq_room'); blk(c,'Gain','neg_room','Gain','-1');
blk(c,'Saturation Dynamic','vq_limit');                               % ports: up, u, lo
% |v| for logging
blk(c,'Product','vq_sq'); blk(c,'Sum','v_sq','Inputs','++'); blk(c,'Sqrt','vmag');
blk(c,'Mux','diag_mux','Inputs','6');
blk(c,'Out1','vd','Port','1'); blk(c,'Out1','vq','Port','2'); blk(c,'Out1','diag','Port','3');
E = { 'iq_cmd/1','iq_cmd_limit/1'; 'wm/1','we/1';
      'id_ref/1','ed/1'; 'id/1','ed/2'; 'iq_cmd_limit/1','eq/1'; 'iq/1','eq/2';
      'ed/1','Kp_d/1'; 'ed/1','Ki_d/1'; 'Ki_d/1','xd_drive/1'; 'xd_drive/1','xd/1';
      'Kp_d/1','PI_d/1'; 'xd/1','PI_d/2';
      'eq/1','Kp_q/1'; 'eq/1','Ki_q/1'; 'Ki_q/1','xq_drive/1'; 'xq_drive/1','xq/1';
      'Kp_q/1','PI_q/1'; 'xq/1','PI_q/2';
      'iq/1','Lq_iq/1'; 'we/1','we_Lq_iq/1'; 'Lq_iq/1','we_Lq_iq/2';
      'id/1','Ld_id/1'; 'Ld_id/1','flux_d/1'; 'psi/1','flux_d/2';
      'we/1','we_flux_d/1'; 'flux_d/1','we_flux_d/2';
      'PI_d/1','vd_raw/1'; 'we_Lq_iq/1','vd_raw/2';
      'PI_q/1','vq_raw/1'; 'we_flux_d/1','vq_raw/2';
      'vd_raw/1','vd_limit/1';
      'vd_limit/1','vd_sq/1'; 'vd_limit/1','vd_sq/2';
      'Vmax_sq/1','vq_room_sq/1'; 'vd_sq/1','vq_room_sq/2'; 'vq_room_sq/1','nonneg/1';
      'nonneg/1','vq_room/1'; 'vq_room/1','neg_room/1';
      'vq_room/1','vq_limit/1'; 'vq_raw/1','vq_limit/2'; 'neg_room/1','vq_limit/3';
      'vd_limit/1','vd_excess/1'; 'vd_raw/1','vd_excess/2'; 'vd_excess/1','Kaw_d/1'; 'Kaw_d/1','xd_drive/2';
      'vq_limit/1','vq_excess/1'; 'vq_raw/1','vq_excess/2'; 'vq_excess/1','Kaw_q/1'; 'Kaw_q/1','xq_drive/2';
      'vq_limit/1','vq_sq/1'; 'vq_limit/1','vq_sq/2'; 'vd_sq/1','v_sq/1'; 'vq_sq/1','v_sq/2'; 'v_sq/1','vmag/1';
      'iq_cmd_limit/1','diag_mux/1'; 'vd_raw/1','diag_mux/2'; 'vq_raw/1','diag_mux/3';
      'vmag/1','diag_mux/4'; 'xd/1','diag_mux/5'; 'xq/1','diag_mux/6';
      'vd_limit/1','vd/1'; 'vq_limit/1','vq/1'; 'diag_mux/1','diag/1'};
wireall(c,E); Simulink.BlockDiagram.arrangeSystem(c);

%% ---------------- root harness ----------------
blk(m,'In1','iq_cmd','Port','1','Interpolate','off');
blk(m,'In1','wm_imposed','Port','2','Interpolate','off');
blk(m,'Gain','inv_J_test','Gain','1/p.J_test');
blk(m,'Integrator','wm_free','InitialCondition','0');
blk(m,'Constant','free_rotor','Value','free_rotor');
blk(m,'Switch','wm_select','Criteria','u2 ~= 0');
blk(m,'Integrator','theta_m','InitialCondition','0');
blk(m,'Mux','log_mux','Inputs','6');
blk(m,'ToWorkspace','stage1_log','VariableName','stage1_log','SaveFormat','Timeseries');
E = { 'iq_cmd/1','FOC_current_ctrl/1';
      'PMSM_dq_manual/1','FOC_current_ctrl/2'; 'PMSM_dq_manual/2','FOC_current_ctrl/3';
      'wm_select/1','FOC_current_ctrl/4';
      'FOC_current_ctrl/1','PMSM_dq_manual/1'; 'FOC_current_ctrl/2','PMSM_dq_manual/2';
      'wm_select/1','PMSM_dq_manual/3';
      'PMSM_dq_manual/3','inv_J_test/1'; 'inv_J_test/1','wm_free/1';
      'wm_free/1','wm_select/1'; 'free_rotor/1','wm_select/2'; 'wm_imposed/1','wm_select/3';
      'wm_select/1','theta_m/1';
      'PMSM_dq_manual/1','log_mux/1'; 'PMSM_dq_manual/2','log_mux/2'; 'PMSM_dq_manual/3','log_mux/3';
      'wm_select/1','log_mux/4'; 'FOC_current_ctrl/1','log_mux/5'; 'FOC_current_ctrl/2','log_mux/6' };
wireall(m,E);
% LOG_COLUMNS: 1 id 2 iq 3 Te 4 wm 5 vd 6 vq | 7 theta_m | 8 iq_ref 9 vd_raw 10 vq_raw 11 vmag 12 xd 13 xq
blk(m,'Mux','log_all','Inputs','3');
wire(m,'log_mux/1','log_all/1'); wire(m,'theta_m/1','log_all/2'); wire(m,'FOC_current_ctrl/3','log_all/3');
wire(m,'log_all/1','stage1_log/1');
Simulink.BlockDiagram.arrangeSystem(m);
set_param(m,'SimulationCommand','update');
save_system(m);
fprintf('Saved %s.slx\n', m);
end

% ---------------- helpers ----------------
function blk(s,type,name,varargin)
switch type
    case {'In1','Constant'},                 lib = 'simulink/Sources/';
    case {'Out1','ToWorkspace'},             lib = 'simulink/Sinks/';
    case {'Integrator'},                     lib = 'simulink/Continuous/';
    case {'Saturation','Saturation Dynamic'},lib = 'simulink/Discontinuities/';
    case {'Mux','Switch'},                   lib = 'simulink/Signal Routing/';
    otherwise,                               lib = 'simulink/Math Operations/';   % Gain Sum Product Sqrt
end
if strcmp(type,'ToWorkspace'), type = 'To Workspace'; end
add_block([lib type],[s '/' name],varargin{:});
end
function wire(s,a,b), add_line(s,a,b,'autorouting','on'); end
function wireall(s,E), for k = 1:size(E,1), wire(s,E{k,1},E{k,2}); end, end

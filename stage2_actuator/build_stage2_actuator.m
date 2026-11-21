function build_stage2_actuator(stage1Model)
% BUILD_STAGE2_ACTUATOR  Builds J2_Actuator.slx: the joint-2 actuator with the port layout of the
% interface guide. Its root IS the actuator, so its contents can be copied into the Gen3 slot.
%
%   Inputs  1 tau_cmd_J [N m]  2 q_J [rad]  3 qd_J [rad/s]      Output 1 tau_J [N m]
%
%   1  torque_to_iq        iq_cmd = tau_cmd_J / (s*N*Kt)
%   2  FOC_current_ctrl    copied from stage 1 (limit +-Imax, |v| <= Vmax, anti-windup)
%   3  PMSM_dq_manual      copied from stage 1 (dq equations -> Te)
%   4  motor shaft         (Jm + J_WG) dwm/dt = Te - (s/N)*tau_J,   dthm/dt = wm
%   5  gear spring         tau_J = Kg*(s*thm/N - q_J) + cg*(s*wm/N - qd_J)
%   FOC and PMSM get the shaft speed wm (not N*qd: the gear is elastic).
%
%   Parameters: struct "act" in the model workspace = j2_params() + Jr, theta_m0, omega_m0.
%   Stage-1 blocks used "p.xxx"; they are renamed to "act.xxx" so they cannot clash with Gen3 variables.
%   Log: j2_act_log (10 columns) iq_cmd iq id vd vq |v| wm Te tau_J deflection
%
%   stage1Model : (optional) full path of J2_PMSM_Stage1.slx, found automatically otherwise
here = fileparts(mfilename('fullpath'));
if nargin < 1, stage1Model = ''; end
stage1Model = find_stage1(here, stage1Model);
[~, src] = fileparts(stage1Model);
m = 'J2_Actuator';
for b = {m, src}, if bdIsLoaded(b{1}), close_system(b{1},0); end, end
load_system(stage1Model);

act = j2_params();
act.Jr = act.N^2*(act.Jm + act.JWG);      % reflected inertia, 0.249 kg m^2 (for ctc_Jr)
act.theta_m0 = 0; act.omega_m0 = 0;       % Gen3 integration sets s*N*q0(2), s*N*dq0(2)

new_system(m);
mw = get_param(m,'ModelWorkspace'); mw.assignin('act', act);
set_param(m,'Solver','ode23t','MaxStep','5e-5','RelTol','1e-6','AbsTol','1e-8','StopTime','1');
set_param(m,'Description',['Joint-2 actuator (stage 2): torque->iq, FOC, dq PMSM, motor shaft, ' ...
    'harmonic-drive spring. Ports: tau_cmd_J, q_J, qd_J -> tau_J.']);

% ---- stage-1 blocks, parameters renamed p. -> act. ----
add_block([src '/FOC_current_ctrl'], [m '/FOC_current_ctrl']);
add_block([src '/PMSM_dq_manual'],   [m '/PMSM_dq_manual']);
close_system(src, 0);
rename_params([m '/FOC_current_ctrl']); rename_params([m '/PMSM_dq_manual']);

% ---- ports ----
add_block('simulink/Sources/In1',[m '/tau_cmd_J'],'Port','1');
add_block('simulink/Sources/In1',[m '/q_J'],'Port','2');
add_block('simulink/Sources/In1',[m '/qd_J'],'Port','3');
add_block('simulink/Sinks/Out1',[m '/tau_J'],'Port','1');

% ---- 1 torque -> iq ----
G = @(n,g) add_block('simulink/Math Operations/Gain',[m '/' n],'Gain',g);
S = @(n,i) add_block('simulink/Math Operations/Sum',[m '/' n],'Inputs',i);
G('torque_to_iq','1/(act.s*act.N*act.Kt)');
% ---- 4 motor shaft ----
S('shaft_torque','+-'); G('gear_reaction','act.s/act.N'); G('inv_Jshaft','1/(act.Jm+act.JWG)');
add_block('simulink/Continuous/Integrator',[m '/omega_m'],'InitialCondition','act.omega_m0');
add_block('simulink/Continuous/Integrator',[m '/theta_m'],'InitialCondition','act.theta_m0');
% ---- 5 gear spring ----
G('theta_to_joint','act.s/act.N'); S('deflection','+-'); G('Kg','act.Kg');
G('omega_to_joint','act.s/act.N'); S('deflection_rate','+-'); G('cg','act.cg');
S('tau_gear','++');
% ---- log ----
add_block('simulink/Signal Routing/Demux',[m '/diag_split'],'Outputs','6');
for k = [2 3 5 6], add_block('simulink/Sinks/Terminator',sprintf('%s/diag_unused%d',m,k)); end
add_block('simulink/Signal Routing/Mux',[m '/log_mux'],'Inputs','10');
add_block('simulink/Sinks/To Workspace',[m '/log_j2_act'],'VariableName','j2_act_log','SaveFormat','Timeseries');

E = { 'tau_cmd_J/1','torque_to_iq/1'; 'torque_to_iq/1','FOC_current_ctrl/1';
      'PMSM_dq_manual/1','FOC_current_ctrl/2'; 'PMSM_dq_manual/2','FOC_current_ctrl/3';
      'omega_m/1','FOC_current_ctrl/4';
      'FOC_current_ctrl/1','PMSM_dq_manual/1'; 'FOC_current_ctrl/2','PMSM_dq_manual/2';
      'omega_m/1','PMSM_dq_manual/3';
      'PMSM_dq_manual/3','shaft_torque/1'; 'tau_gear/1','gear_reaction/1'; 'gear_reaction/1','shaft_torque/2';
      'shaft_torque/1','inv_Jshaft/1'; 'inv_Jshaft/1','omega_m/1'; 'omega_m/1','theta_m/1';
      'theta_m/1','theta_to_joint/1'; 'theta_to_joint/1','deflection/1'; 'q_J/1','deflection/2';
      'deflection/1','Kg/1'; 'Kg/1','tau_gear/1';
      'omega_m/1','omega_to_joint/1'; 'omega_to_joint/1','deflection_rate/1'; 'qd_J/1','deflection_rate/2';
      'deflection_rate/1','cg/1'; 'cg/1','tau_gear/2';
      'tau_gear/1','tau_J/1';
      'FOC_current_ctrl/3','diag_split/1';
      'diag_split/2','diag_unused2/1'; 'diag_split/3','diag_unused3/1';
      'diag_split/5','diag_unused5/1'; 'diag_split/6','diag_unused6/1';
      'diag_split/1','log_mux/1'; 'PMSM_dq_manual/2','log_mux/2'; 'PMSM_dq_manual/1','log_mux/3';
      'FOC_current_ctrl/1','log_mux/4'; 'FOC_current_ctrl/2','log_mux/5'; 'diag_split/4','log_mux/6';
      'omega_m/1','log_mux/7'; 'PMSM_dq_manual/3','log_mux/8'; 'tau_gear/1','log_mux/9';
      'deflection/1','log_mux/10'; 'log_mux/1','log_j2_act/1' };
for k = 1:size(E,1), add_line(m, E{k,1}, E{k,2}, 'autorouting','on'); end
Simulink.BlockDiagram.arrangeSystem(m);
save_system(m, fullfile(here,[m '.slx']));
close_system(m, 0);
fprintf('Saved %s.slx  (Jr = %.3f kg m^2, Kg = %.3g N m/rad, s = %d)\n', fullfile(here,m), act.Jr, act.Kg, act.s);
end

function rename_params(sys)
% Stage-1 blocks reference p.xxx; switch them to act.xxx (whole-word p. only)
blks = find_system(sys, 'LookUnderMasks','all', 'FollowLinks','on', 'Type','Block');
names = {'Gain','Value','UpperLimit','LowerLimit','InitialCondition'};
for k = 1:numel(blks)
    dp = get_param(blks{k}, 'DialogParameters');
    if ~isstruct(dp), continue; end
    for n = names
        if isfield(dp, n{1})
            v = get_param(blks{k}, n{1});
            w = regexprep(v, '(?<![\w.])p\.', 'act.');
            if ~strcmp(v, w), set_param(blks{k}, n{1}, w); end
        end
    end
end
end

function res = run_p2p_ideal(inputMat, model, tag, wsVars)
% RUN_P2P_IDEAL  Runs the slow point-to-point motion and evaluates it (stage 3, item 2).
%   Default: IDEAL torque reference case (Gen3_J2_slot, J2_Actuator = pass-through).
%   The same function evaluates other models on the same input (payload CTC, actuator, ...).
%
%   inputMat : MAT from make_p2p_trajectory (default p2p_input.mat next to this file)
%   model    : model name in this folder            (default 'Gen3_J2_slot')
%   tag      : name used in the output files       (default 'ideal')
%   wsVars   : struct of model-workspace overrides for this run only, e.g.
%              struct('robotPayload', make_payload_robot(robotToTest, 0.12))
%   Output (results/, names use tag; listed for tag = 'ideal'):
%     p2p_ideal_metrics.csv   per joint: tracking error, peak speed vs limit, peak/RMS torque
%     p2p_ideal.png           joints 2 and 4: reference vs actual, speed, torque
%     joint2_p2p_trace.csv    same columns as data/joint2_baseline_trace.csv (for sizing)
%     p2p_ideal_out.mat       t, q, qd, tau, q_ref (input for analyze_inertia_p2p)
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(inputMat), inputMat = fullfile(here,'p2p_input.mat'); end
if nargin < 2 || isempty(model), model = 'Gen3_J2_slot'; end
if nargin < 3 || isempty(tag), tag = 'ideal'; end
if nargin < 4, wsVars = struct(); end
S = load(inputMat);
m = model;
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
T = S.p2p_t_end;

in = Simulink.SimulationInput(m);
for v = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'}
    in = in.setVariable(v{1}, S.(v{1}));
end
wf = fieldnames(wsVars);
for k = 1:numel(wf), in = in.setVariable(wf{k}, wsVars.(wf{k}), 'Workspace', m); end
in = in.setModelParameter('StopTime', num2str(T), ...
    'OutputOption','AdditionalOutputTimes', 'OutputTimes', sprintf('0:0.001:%g', T));
out = sim(in);

t   = out.q_actual.Time;
q   = as_rows(out.q_actual.Data);
qd  = as_rows(out.qd_actual.Data);
tau = resample_to(out.tau_applied, t);
tcm = resample_to(out.tau_joint, t);
qr  = resample_to(S.q_cmd, t);
qdr = resample_to(S.qd_cmd, t);
e = qr - q; ed = qdr - qd;

% ---- per-joint metrics ----
vlim = 1.36;                                   % rad/s, Gen3 large actuator no-load (13 RPM)
twrms = @(x) sqrt(trapz(t, x.^2) / (t(end) - t(1)));
J = (1:7)'; rows = zeros(7,8);
for k = 1:7
    rows(k,:) = [max(abs(e(:,k))), twrms(e(:,k)), max(abs(ed(:,k))), ...
                 max(abs(qd(:,k))), max(abs(qd(:,k)))/vlim*100, ...
                 max(abs(tau(:,k))), twrms(tau(:,k)), max(abs(tcm(:,k) - tau(:,k)))];
end
M = array2table([J rows], 'VariableNames', {'Joint','MaxPosErr_rad','RmsPosErr_rad', ...
    'MaxVelErr_rad_s','PeakSpeed_rad_s','PeakSpeed_pct_of_1p36','PeakTorque_Nm', ...
    'RmsTorque_Nm','Cmd_minus_Applied_Nm'});
disp(M);
rd = fullfile(here,'results'); if ~exist(rd,'dir'), mkdir(rd); end
writetable(M, fullfile(rd,['p2p_' tag '_metrics.csv']));

% ---- joint 2 trace in the baseline CSV format ----
P2 = tau(:,2).*qd(:,2);
tr = table(t, tcm(:,2), tau(:,2), qd(:,2), q(:,2), P2, 'VariableNames', ...
    {'Time_s','TorqueBeforeLimit_Nm','TorqueApplied_Nm','ActualVelocity_rad_s', ...
     'ActualPosition_rad','MechanicalPower_W'});
if strcmp(tag,'ideal'), trName = 'joint2_p2p_trace.csv'; else, trName = ['joint2_p2p_trace_' tag '.csv']; end
writetable(tr, fullfile(rd,trName));
save(fullfile(rd,['p2p_' tag '_out.mat']), 't','q','qd','tau','tcm','qr','qdr');

% ---- figure: joints 2 and 4 ----
f = figure('Visible','off','Position',[50 50 1000 850]); tiledlayout(4,1);
nexttile; plot(t,qr(:,2),'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(t,q(:,2),'--');
plot(t,qr(:,4),'LineWidth',3,'Color',[1 .8 .6]); plot(t,q(:,4),'--');
ylabel('q (rad)'); legend('q_2 ref','q_2','q_4 ref','q_4','Location','best'); grid on;
title(sprintf('Slow point-to-point motion: %s (%s)', strrep(tag,'_','\_'), strrep(m,'_','\_')));
nexttile; plot(t, 1e3*e(:,2), t, 1e3*e(:,4)); ylabel('error (mrad)'); legend('joint 2','joint 4'); grid on;
nexttile; plot(t, qd(:,2), t, qd(:,4)); yline([vlim -vlim],'r:'); ylabel('speed (rad/s)');
legend('joint 2','joint 4','\pm1.36 limit'); grid on;
nexttile; plot(t, tau(:,2), t, tau(:,4)); ylabel('torque (N m)'); xlabel('time (s)');
legend('joint 2','joint 4'); grid on;
exportgraphics(f, fullfile(rd,['p2p_' tag '.png']), 'Resolution',130); close(f);

res = struct('metrics',M,'t',t,'q',q,'qd',qd,'tau',tau,'e',e,'tag',tag,'model',m);
fprintf('[%s] Joint 2: max error %.3g mrad, peak speed %.3f rad/s (%.0f%% of limit), peak torque %.2f N m\n', ...
    tag, 1e3*M.MaxPosErr_rad(2), M.PeakSpeed_rad_s(2), M.PeakSpeed_pct_of_1p36(2), M.PeakTorque_Nm(2));
end

function d = as_rows(d)
% timeseries data [N x 7] or [7 x 1 x N] -> [N x 7]
if ndims(d) == 3, d = reshape(permute(d,[3 1 2]), size(d,3), []); end
if size(d,2) ~= 7 && size(d,1) == 7, d = d.'; end
end

function y = resample_to(ts, t)
% timeseries -> [numel(t) x 7] on time grid t (duplicate time stamps dropped, last value kept)
[tu, iu] = unique(ts.Time, 'last');
d = as_rows(ts.Data);
y = interp1(tu, d(iu,:), t, 'linear', 'extrap');
end

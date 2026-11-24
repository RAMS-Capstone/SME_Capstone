function R = test_stage3_ctc_jr
% TEST_STAGE3_CTC_JR  Checks the reflected-inertia option of the CTC (Gen3_J2_ctc.slx).
%
%   Run A  ctc_Jr = 0 (off)           must equal Gen3_J2_payload exactly
%   Run B  ctc_Jr(2) = Jr = 0.249     CTC adds Jr*a on joint 2
%
%   Check 1  A vs results/p2p_payload_out.mat (saved by test_stage3_payload): q and CTC torque equal.
%   Check 2  For A and B, the logged CTC torque is rebuilt in MATLAB from the logged states:
%              tau = inverseDynamics(robotPayload, q, qd, a) + ctc_Jr .* a
%              a   = qdd_ref + Kp*(q_ref - q) + Kd*(qd_ref - qd)
%            -> proves the Simulink CTC computes exactly (M + diag(ctc_Jr))*a + C*qd + g.
%   Info     Run B has NO rotor in the plant (slot is still a pass-through), so its tracking error
%            shows what a CTC that assumes a rotor does to a robot without one (the mirror of the
%            "CTC does not know the actuator" case of design decision 2).
%
%   Needs: Gen3_J2_ctc.slx (build_stage3_ctc_jr), p2p_input.mat, results/p2p_payload_out.mat
%   Output: results/ctc_jr_check.csv, results/ctc_jr_check.png
here = fileparts(mfilename('fullpath'));
m = 'Gen3_J2_ctc';
ensure_stage3_model(m);
p2pMat = ensure_p2p_input(); S = load(p2pMat);
Jr = 100^2*(170e-7 + 0.079e-4);           % N^2*(Jm + J_WG), stage1_motor/j2_params.m
JrVec = [0 Jr 0 0 0 0 0]';

fprintf('[1/2] slow motion, ctc_Jr off ... '); tk = tic;
A = run_p2p_ideal(p2pMat, m, 'ctc_off');
fprintf('[1/2] done (%.0f s)\n', toc(tk));
fprintf('[2/2] slow motion, ctc_Jr(2) = %.3f ... ', Jr); tk = tic;
B = run_p2p_ideal(p2pMat, m, 'ctc_jr_on', struct('ctc_Jr', JrVec));
fprintf('[2/2] done (%.0f s)\n', toc(tk));
rd = fullfile(here,'results');
LA = load(fullfile(rd,'p2p_ctc_off_out.mat')); LB = load(fullfile(rd,'p2p_ctc_jr_on_out.mat'));

% ---- check 1: option off == payload model ----
pm = fullfile(rd,'p2p_payload_out.mat');
if isfile(pm)
    LP = load(pm);
    dq = max(abs(LA.q - rs(LP.t, LP.q, LA.t)), [], 'all');
    dt = max(abs(LA.tcm - rs(LP.t, LP.tcm, LA.t)), [], 'all');
else
    dq = NaN; dt = NaN; fprintf('Check 1 skipped: %s not found.\n', pm);
end

% ---- check 2: rebuild the CTC torque ----
mw = get_param(m,'ModelWorkspace');
rb = copy(mw.getVariable('robotPayload')); rb.DataFormat = 'row';   % ball held during the whole p2p
Kp = str2double(get_param([m '/Computed Torque Controller/Kp'],'Gain'));
Kd = str2double(get_param([m '/Computed Torque Controller/Kd'],'Gain'));
[errA, aA] = rebuild(LA, S, rb, Kp, Kd, zeros(7,1));
[errB, aB] = rebuild(LB, S, rb, Kp, Kd, JrVec);

% ---- effect of the option ----
dTau2 = LB.tcm(:,2) - rs(LA.t, LA.tcm(:,2), LB.t);
eB = LB.qr - LB.q;
R = struct( ...
  'off_vs_payload_q_maxdiff_rad', dq, 'off_vs_payload_tau_maxdiff_Nm', dt, ...
  'rebuild_off_maxdiff_Nm', errA, 'rebuild_on_maxdiff_Nm', errB, ...
  'Jr_kgm2', Jr, 'Jr_times_a2_peak_Nm', Jr*max(abs(aB(:,2))), ...
  'tau2_on_minus_off_peak_Nm', max(abs(dTau2)), ...
  'on_maxErr_j2_mrad', 1e3*max(abs(eB(:,2))), 'on_maxErr_j4_mrad', 1e3*max(abs(eB(:,4))), ...
  'on_maxErr_all_mrad', 1e3*max(abs(eB),[],'all'));
disp(R);
writetable(struct2table(R), fullfile(rd,'ctc_jr_check.csv'));

f = figure('Visible','off','Position',[50 50 1000 650]); tiledlayout(2,1);
nexttile; plot(LA.t, LA.tcm(:,2), 'LineWidth',2); hold on; plot(LB.t, LB.tcm(:,2), '--');
ylabel('CTC \tau_2 (N m)'); grid on; legend('ctc\_Jr off','ctc\_Jr(2) = 0.249','Location','best');
title('Reflected-inertia option of the CTC (ideal plant, no rotor yet)');
yyaxis right; plot(LB.t, dTau2, ':', 'DisplayName','on - off (right axis)'); ylabel('on - off (N m)');
nexttile; plot(LB.t, 1e3*eB(:,2), LB.t, 1e3*eB(:,4)); grid on; ylabel('error (mrad)'); xlabel('time (s)');
legend('joint 2','joint 4'); title('Tracking error when the CTC assumes a rotor the plant does not have');
exportgraphics(f, fullfile(rd,'ctc_jr_check.png'), 'Resolution',130); close(f);

tol = 5e-3;   % N m (logged signals between solver steps are interpolated: allow 0.05% of the 11 N m peak)
ok = errA < tol && errB < tol && (isnan(dq) || (dq < 1e-9 && dt < 1e-9));
assert(ok, ['CTC option check failed: rebuild off %.3g / on %.3g N m, off vs payload q %.3g rad, ' ...
            'tau %.3g N m'], errA, errB, dq, dt);
fprintf('PASS: CTC = (M + diag(ctc_Jr))*a + C*qd + g; option off is identical to Gen3_J2_payload.\n');
end

function [err, a] = rebuild(L, S, rb, Kp, Kd, JrVec)
% max |logged CTC torque - recomputed|, evaluated every 10 ms (+ the acceleration command a)
[tu, iu] = unique(L.t, 'last');
ts = (tu(1):0.01:tu(end))';
q   = rs(tu, L.q(iu,:), ts);   qd  = rs(tu, L.qd(iu,:), ts);
qr  = rs(tu, L.qr(iu,:), ts);  qdr = rs(tu, L.qdr(iu,:), ts);
tc  = rs(tu, L.tcm(iu,:), ts);
[tq, iq] = unique(S.qdd_cmd.Time, 'last'); Qdd = as_rows(S.qdd_cmd.Data);
qddr = rs(tq, Qdd(iq,:), ts);
a = qddr + Kp*(qr - q) + Kd*(qdr - qd);
tex = zeros(size(q));
for k = 1:numel(ts)
    tex(k,:) = inverseDynamics(rb, q(k,:), qd(k,:), a(k,:)) + (JrVec.' .* a(k,:));
end
err = max(abs(tc - tex), [], 'all');
end

function y = rs(t, d, tq)
[tu, iu] = unique(t, 'last');
y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end

function d = as_rows(d)
if ndims(d) == 3, d = reshape(permute(d,[3 1 2]), size(d,3), []); end
if size(d,2) ~= 7 && size(d,1) == 7, d = d.'; end
end

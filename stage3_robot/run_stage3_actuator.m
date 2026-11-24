function R = run_stage3_actuator(opts)
% RUN_STAGE3_ACTUATOR  Ideal torque vs the real joint-2 actuator on the slow point-to-point motion.
%
%   Cases (same p2p_input.mat: joint 2 +0.8 rad, joint 4 -0.8 rad, ball held, 8.5 s):
%     ideal     Gen3_J2_ctc, ctc_Jr = 0     joint 2 gets exactly the CTC torque (reference)
%     act_jr    Gen3_J2_act, ctc_Jr(2) = Jr actuator in the slot, CTC knows the rotor (design decision 2)
%     act_nojr  Gen3_J2_act, ctc_Jr = 0     actuator in the slot, CTC does NOT know the rotor
%   Every actuator run starts aligned (j2_actuator_init): tau_J(0) equals the first CTC torque.
%
%   Checks (the run stops with an error if one fails, after writing the results):
%     start     max |tau_J - tau_cmd| over the first 50 ms < 0.5 N m   (motor shaft aligned)
%     current   peak |iq_cmd| < Imax (20.9 A)                           (no current saturation)
%     voltage   peak |v_dq|   < Vmax (12.47 V)
%     gear      peak |tau_J|  < 70 N m (CSG-17-100 repeated peak torque)
%   Reported (no threshold): tracking error per joint, deviation from the ideal run, CTC command
%   minus applied torque, motor speed, gear deflection.
%
%   opts fields (all optional)
%     reuseIdeal  true   reuse results/p2p_ctc_off_out.mat (test_stage3_ctc_jr) if it exists
%     cases       {'act_jr','act_nojr'}
%     solver      model parameters for the actuator runs, merged into the default
%                 ode23t / MaxStep 1e-3 / RelTol 1e-4 (fastest in time_check_actuator, 2026-11-24:
%                 ~34 ms per step, about 7 min for 8.5 s)
%     convergence true   also run act_jr for the first 2 s with MaxStep 2e-4 / RelTol 1e-5 and
%                 report how much q2 and tau_J change (results/stage3_actuator_convergence.csv)
%     stopTime    simulated seconds of the actuator runs (default: whole motion, 8.5 s), e.g. 2
%                 for a quick look at the hold and the first part of the move
%   Needs  : Gen3_J2_ctc.slx, Gen3_J2_act.slx (build_stage3_actuator), p2p_input.mat (made if missing)
%   Output : results/stage3_actuator_compare.csv, results/stage3_actuator_compare.png,
%            results/stage3_actuator_convergence.csv,
%            results/p2p_<case>_metrics.csv / _out.mat (run_p2p_ideal)
here = fileparts(mfilename('fullpath'));
if nargin < 1, opts = struct(); end
d = struct('reuseIdeal', true, 'cases', {{'act_jr','act_nojr'}}, 'solver', struct(), 'stopTime', [], ...
           'convergence', true);
fn = fieldnames(d);
for k = 1:numel(fn), if ~isfield(opts,fn{k}), opts.(fn{k}) = d.(fn{k}); end, end
ensure_stage3_model('Gen3_J2_act');      % builds the missing models on a fresh checkout
p2pMat = ensure_p2p_input();
S = load(p2pMat);
rd = fullfile(here,'results');

% ---------------- ideal reference ----------------
idealMat = fullfile(rd,'p2p_ctc_off_out.mat');
nRuns = numel(opts.cases) + ~(opts.reuseIdeal && isfile(idealMat));
ir = 0;
if opts.reuseIdeal && isfile(idealMat)
    fprintf('ideal: reused %s\n', idealMat);
else
    ir = 1; fprintf('[1/%d] ideal (Gen3_J2_ctc, ctc_Jr off) ... ', nRuns); tk = tic;
    run_p2p_ideal(p2pMat, 'Gen3_J2_ctc', 'ctc_off');
    fprintf('done (%.0f s)\n', toc(tk));
end
I = load(idealMat);

% ---------------- actuator runs ----------------
m = 'Gen3_J2_act';
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
mw = get_param(m,'ModelWorkspace');
act0 = mw.getVariable('act'); JrOn = mw.getVariable('ctc_Jr');
simP = struct('SolverName','ode23t', 'MaxStep','1e-3', 'RelTol','1e-4');
f = fieldnames(opts.solver); for k = 1:numel(f), simP.(f{k}) = opts.solver.(f{k}); end
simP.Refine = '1';     % solver step is already small: no output refinement
if ~isempty(opts.stopTime), simP.StopTime = num2str(opts.stopTime); end
C = struct([]);
for ic = 1:numel(opts.cases)
    tag = opts.cases{ic};
    switch tag
        case 'act_jr',   jr = JrOn;
        case 'act_nojr', jr = zeros(7,1);
        otherwise, error('Unknown case %s', tag);
    end
    [act, tau0] = j2_actuator_init(m, S, jr);
    fprintf('[%d/%d] %s (ctc_Jr(2) = %.3f, tau0(2) = %.3f N m) ... ', ir+ic, nRuns, tag, jr(2), tau0(2));
    tk = tic;
    r = run_p2p_ideal(p2pMat, m, tag, struct('act', act, 'ctc_Jr', jr), simP);
    fprintf('done (%.0f s)\n', toc(tk));
    assert(~isempty(r.act_log), 'j2_act_log was not logged: is the actuator in the slot?');
    r.jr = jr; r.runtime = toc(tk);
    if isempty(C), C = r; else, C(end+1) = r; end %#ok<AGROW>
end

% ---------------- metrics ----------------
rows = {}; okAll = true; msgs = {};
eI = I.qr - I.q;
rows(end+1,:) = {'ideal', 0, 1e3*max(abs(eI(:,2))), 1e3*max(abs(eI(:,4))), 1e3*max(abs(eI),[],'all'), ...
    0, 0, max(abs(I.tau(:,2))), 0, NaN, NaN, NaN, NaN, NaN, NaN};
for ic = 1:numel(C)
    r = C(ic); t = r.t; L = r.act_log; ta = r.act_t;
    dev = r.q - rs(I.t, I.q, t);                         % actuator run minus ideal run
    tauJ = L(:,9); tcm2 = rs(t, r.tcm(:,2), ta);
    st = ta <= 0.05;
    startErr = max(abs(tauJ(st) - tcm2(st)));
    iqPk = max(abs(L(:,1))); vPk = max(abs(L(:,6))); wPk = max(abs(L(:,7)))*60/(2*pi);
    tauPk = max(abs(tauJ)); defPk = max(abs(L(:,10)));
    rows(end+1,:) = {r.tag, r.jr(2), 1e3*max(abs(r.e(:,2))), 1e3*max(abs(r.e(:,4))), ...
        1e3*max(abs(r.e),[],'all'), 1e3*max(abs(dev(:,2))), 1e3*max(abs(dev),[],'all'), ...
        tauPk, max(abs(tauJ - tcm2)), startErr, iqPk, vPk, wPk, 1e3*defPk, r.runtime}; %#ok<AGROW>
    chk = { startErr < 0.5, sprintf('%s: start |tau_J - tau_cmd| = %.3g N m (>= 0.5)', r.tag, startErr);
            iqPk < act0.Imax, sprintf('%s: iq_cmd reaches the %.1f A limit', r.tag, act0.Imax);
            vPk < act0.Vmax,  sprintf('%s: |v| reaches the %.2f V limit', r.tag, act0.Vmax);
            tauPk < 70,       sprintf('%s: gear torque %.1f N m above the 70 N m repeated peak', r.tag, tauPk) };
    for k = 1:size(chk,1), if ~chk{k,1}, okAll = false; msgs{end+1} = chk{k,2}; end, end %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'Case','ctc_Jr2_kgm2','MaxErr_j2_mrad','MaxErr_j4_mrad', ...
    'MaxErr_all_mrad','DevFromIdeal_j2_mrad','DevFromIdeal_all_mrad','PeakTauJ2_Nm', ...
    'MaxCmdMinusApplied_j2_Nm','StartMismatch_Nm','PeakIqCmd_A','PeakV_V','PeakMotorSpeed_rpm', ...
    'PeakGearDeflection_mrad','Runtime_s'});
disp(T);
writetable(T, fullfile(rd,'stage3_actuator_compare.csv'));

% ---------------- figure ----------------
f = figure('Visible','off','Position',[50 50 1100 1000]); tiledlayout(5,1);
nexttile; plot(I.t, 1e3*eI(:,2), 'LineWidth',2); hold on;
for ic = 1:numel(C), plot(C(ic).t, 1e3*C(ic).e(:,2)); end
ylabel('e_2 (mrad)'); grid on; legend(['ideal', {C.tag}], 'Interpreter','none','Location','best');
title('Joint-2 actuator in Gen3, slow point-to-point: ideal torque vs actuator');
nexttile; plot(I.t, 1e3*eI(:,4), 'LineWidth',2); hold on;
for ic = 1:numel(C), plot(C(ic).t, 1e3*C(ic).e(:,4)); end
ylabel('e_4 (mrad)'); grid on;
nexttile; plot(I.t, I.tau(:,2), 'LineWidth',2); hold on;
for ic = 1:numel(C), plot(C(ic).act_t, C(ic).act_log(:,9)); end
ylabel('\tau_2 applied (N m)'); grid on;
nexttile; hold on;
for ic = 1:numel(C), plot(C(ic).act_t, C(ic).act_log(:,2)); end
yline([act0.Imax -act0.Imax],'r:'); ylabel('i_q (A)'); grid on;
nexttile; hold on;
for ic = 1:numel(C), plot(C(ic).act_t, 1e3*C(ic).act_log(:,10)); end
ylabel('gear deflection (mrad)'); xlabel('time (s)'); grid on;
exportgraphics(f, fullfile(rd,'stage3_actuator_compare.png'), 'Resolution',130); close(f);

% ---------------- step-size check: act_jr again with a 5x smaller step ----------------
Cv = [];
ia = find(strcmp({C.tag}, 'act_jr'), 1);
if opts.convergence && ~isempty(ia)
    fine = simP; fine.MaxStep = '2e-4'; fine.RelTol = '1e-5';
    tEnd = 2; if ~isempty(opts.stopTime), tEnd = min(tEnd, opts.stopTime); end
    fine.StopTime = num2str(tEnd);
    [act, ~] = j2_actuator_init(m, S, JrOn);
    fprintf('[check] act_jr, first %g s, MaxStep 2e-4 RelTol 1e-5 ... ', tEnd); tk = tic;
    F = run_p2p_ideal(p2pMat, m, 'act_jr_fine', struct('act', act, 'ctc_Jr', JrOn), fine);
    fprintf('done (%.0f s)\n', toc(tk));
    a = C(ia); tg = (0:1e-3:tEnd)';
    dq2  = 1e3*max(abs(rs(a.t, a.q(:,2), tg) - rs(F.t, F.q(:,2), tg)));
    dq   = 1e3*max(abs(rs(a.t, a.q, tg) - rs(F.t, F.q, tg)), [], 'all');
    dtau = max(abs(rs(a.act_t, a.act_log(:,9), tg) - rs(F.act_t, F.act_log(:,9), tg)));
    diq  = max(abs(rs(a.act_t, a.act_log(:,2), tg) - rs(F.act_t, F.act_log(:,2), tg)));
    Cv = table(tEnd, string(simP.MaxStep), dq2, dq, dtau, diq, 'VariableNames', {'Window_s', ...
        'MaxStep_used','DiffQ2_mrad','DiffQall_mrad','DiffTauJ_Nm','DiffIq_A'});
    disp(Cv); writetable(Cv, fullfile(rd,'stage3_actuator_convergence.csv'));
    if dq < 0.01
        fprintf('Step size OK: q changes by %.2g mrad with a 5x smaller step (< 0.01 mrad).\n', dq);
    else
        warning('Step size matters: q changes by %.3g mrad with a 5x smaller step. Use a smaller MaxStep.', dq);
    end
end

R = struct('table', T, 'cases', C, 'convergence', Cv);
assert(okAll, 'Stage-3 actuator check failed:\n  %s', strjoin(msgs, '\n  '));
fprintf('PASS: actuator starts aligned, stays inside current, voltage and gear limits.\n');
end

function y = rs(t, d, tq)
[tu, iu] = unique(t, 'last');
y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end

function results = test_stage2_actuator
% TEST_STAGE2_ACTUATOR  The four bench tests of the interface guide, compared with the Python
% reference (verify_stage2.py -> stage2_reference.json, independent of Simulink).
%
%   T1 static      joint held, tau_cmd 10 N m step   -> tau_J -> 10 N m, deflection -> 10/Kg = 0.714 mrad
%   T2 sign        T1 with s = +1                    -> tau_J identical (only thm, wm, iq change sign)
%   T3 limit       joint held, tau_cmd 100 N m       -> iq_cmd <= 20.9 A, tau_J -> N*Kt*Imax = 91.2 N m
%   T4 pendulum    J_L = 1.2 kg m^2 + gravity, quintic 0 -> 0.8 rad, same command as an ideal-torque load
%   T4b            T4 with feed-forward Jr*qdd_ref added to the actuator command (what ctc_Jr does)
%   Every run: |v| <= Vmax.
%   Output: stage2_test_results.csv, stage2_tests.png (this folder)
here = fileparts(mfilename('fullpath'));
tb = 'J2_Actuator_TestBench';
assert(isfile(fullfile(here,[tb '.slx'])), 'Run build_stage2_actuator and build_stage2_testbench first.');
ref = jsondecode(fileread(fullfile(here,'stage2_reference.json')));
if bdIsLoaded(tb), close_system(tb,0); end
load_system(fullfile(here,[tb '.slx']));
mw = get_param(tb,'ModelWorkspace');
act = mw.getVariable('act'); bench = mw.getVariable('bench');

rows = {};
% ---------------- T1, T2, T3: joint held ----------------
fprintf('[1/5] T1 static 10 N m ... ');  [t1, L1] = run_step(tb, act, bench, 10, -1);  fprintf('done\n');
fprintf('[2/5] T2 sign s = +1 ... ');    [t2, L2] = run_step(tb, act, bench, 10, +1);  fprintf('done\n');
fprintf('[3/5] T3 limit 100 N m ... ');  [t3, L3] = run_step(tb, act, bench, 100, -1); fprintf('done\n');
tl = @(t,y) mean(y(t >= 0.5));
rows = add(rows,'T1','tauJ_tail_Nm',      tl(t1,L1(:,9)),  ref.T1_static.tauJ_tail, 0.01);
rows = add(rows,'T1','deflection_tail_rad',tl(t1,L1(:,10)), ref.T1_static.defl_expected, 1e-6);
rows = add(rows,'T1','tauJ_peak_Nm',      max(L1(:,9)),    ref.T1_static.tauJ_peak, 0.03*ref.T1_static.tauJ_peak);
rows = add(rows,'T1','iq_tail_A',         tl(t1,L1(:,2)),  ref.T1_static.iq_tail, 0.01);
L2i = interp1u(t2, L2, t1);
rows = add(rows,'T2','tauJ_maxdiff_vs_T1_Nm', max(abs(L2i(:,9) - L1(:,9))), 0, 1e-6);
rows = add(rows,'T2','iq_tail_A (sign flips)', tl(t2,L2(:,2)), ref.T2_sign.iq_tail_s_plus1, 0.01);
rows = add(rows,'T3','iq_cmd_max_A',      max(abs(L3(:,1))), act.Imax, 1e-9);
rows = add(rows,'T3','tauJ_tail_Nm',      tl(t3,L3(:,9)),  ref.T3_limit.tauJ_expected, 0.1);

% ---------------- T4: pendulum ----------------
fprintf('[4/5] T4 pendulum, same command ... '); [t4, L4, B4] = run_pend(tb, act, bench, false); fprintf('done\n');
fprintf('[5/5] T4b pendulum, + Jr feed-forward ... '); [t5, L5, B5] = run_pend(tb, act, bench, true); fprintf('done\n');
e4 = B4(:,1) - B4(:,3); e5 = B5(:,1) - B5(:,3);
rows = add(rows,'T4', 'max_q_minus_ideal_mrad', 1e3*max(abs(e4)), ref.T4_pendulum.max_q_minus_ideal_mrad, ...
           0.15*ref.T4_pendulum.max_q_minus_ideal_mrad);
rows = add(rows,'T4', 'iq_peak_A',  max(abs(L4(:,2))), ref.T4_pendulum.iq_peak, 0.05*ref.T4_pendulum.iq_peak);
rows = add(rows,'T4b','max_q_minus_ideal_mrad', 1e3*max(abs(e5)), ref.T4_pendulum_aware.max_q_minus_ideal_mrad, 0.5);
vpk = max([L1(:,6); L2(:,6); L3(:,6); L4(:,6); L5(:,6)]);
rows = add(rows,'all','vmag_peak_V (<= Vmax)', vpk, act.Vmax, inf);
rows{end,5} = vpk <= act.Vmax*(1+1e-9);

results = cell2table(rows, 'VariableNames', {'Test','Metric','Simulink','Python','Pass'});
disp(results); writetable(results, fullfile(here,'stage2_test_results.csv'));

f = figure('Visible','off','Position',[50 50 1000 900]); tiledlayout(3,1);
nexttile; plot(t1,L1(:,9), t2,L2(:,9),'--', t3,L3(:,9)); grid on; ylabel('\tau_J (N m)');
legend('T1 10 N m, s = -1','T2 10 N m, s = +1','T3 100 N m (current-limited)','Location','east');
title('Joint held: torque through the gear spring');
nexttile; plot(t4, 1e3*e4, t5, 1e3*e5, '--'); grid on; ylabel('q - q_{ideal} (mrad)');
legend('same command as ideal','+ J_r feed-forward'); title('Pendulum: actuator vs ideal torque');
nexttile; plot(t4, L4(:,2), t4, L4(:,6)); yline(act.Vmax,'r:'); grid on; xlabel('time (s)');
legend('i_q (A)','|v| (V)','V_{max}'); title('Pendulum: current and voltage (same command)');
exportgraphics(f, fullfile(here,'stage2_tests.png'), 'Resolution',130); close(f);
close_system(tb, 0);
assert(all(results.Pass), 'Some stage-2 test differs from the reference (see the table).');
fprintf('PASS: J2_Actuator meets all four bench tests.\n');
end

% ======================================================================
function [t, L] = run_step(tb, act, bench, a, s)
T = 0.6;
u = setinterpmethod(timeseries([0; a; a], [0; 0.01; T]), 'zoh');
z = setinterpmethod(timeseries([0; 0], [0; T]), 'zoh');
act.s = s; bench.fixed = 1;
[t, L] = simulate(tb, act, bench, u, z, T);
end

function [t, L, B] = run_pend(tb, act, bench, aware)
T = 5; tc = (0:1e-3:T)';
qr = zeros(size(tc)); qdr = qr; qddr = qr;
for k = 1:numel(tc)
    x = min(max((tc(k) - 0.5)/3, 0), 1);
    qr(k)   = 0.8*(10*x^3 - 15*x^4 + 6*x^5);
    qdr(k)  = 0.8*(30*x^2 - 60*x^3 + 30*x^4)/3;
    qddr(k) = 0.8*(60*x - 180*x^2 + 120*x^3)/9;
end
base = bench.JL*qddr + bench.b*qdr + bench.G*sin(qr);
cmdAct = base + aware*act.Jr*qddr;
bench.fixed = 0;
[t, L, B] = simulate(tb, act, bench, timeseries(cmdAct, tc), timeseries(base, tc), T);
end

function [t, L, B] = simulate(tb, act, bench, uAct, uIdeal, T)
ds = Simulink.SimulationData.Dataset;
ds = ds.addElement(uAct, 'tau_cmd_act'); ds = ds.addElement(uIdeal, 'tau_cmd_ideal');
in = Simulink.SimulationInput(tb); in = in.setExternalInput(ds);
in = in.setVariable('act', act, 'Workspace', tb);
in = in.setVariable('bench', bench, 'Workspace', tb);
in = in.setModelParameter('StopTime', num2str(T));
out = sim(in);
t = out.j2_act_log.Time; L = rows2(out.j2_act_log.Data);
B = interp1u(out.bench_log.Time, rows2(out.bench_log.Data), t);
end

function d = rows2(d)
if ndims(d) == 3, d = reshape(permute(d,[3 1 2]), size(d,3), []); end
end

function y = interp1u(t, d, tq)
[tu, iu] = unique(t, 'last');
y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end

function rows = add(rows, test, metric, v, r, tol)
rows(end+1,:) = {test, metric, v, r, abs(v - r) <= tol};
end

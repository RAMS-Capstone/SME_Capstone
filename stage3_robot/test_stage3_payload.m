function R = test_stage3_payload(throwMat)
% TEST_STAGE3_PAYLOAD  Does the payload-aware CTC remove the hold error, and does it switch at release?
%
%   Part 1 - slow point-to-point (ball held all the time, p2p_input.mat):
%     ideal          Gen3_J2_slot     CTC without ball           (the case with the 7-16 mrad hold error)
%     payload        Gen3_J2_payload  CTC with the true 0.1 kg
%     payload_0p12   Gen3_J2_payload  CTC set to 0.12 kg        (wrong payload setting, +20 %)
%     Pass: 'payload' max error < 0.5 mrad on every joint.
%   Part 2 - throw (ball released at t_release_plan; needs the baseline input MAT):
%     slot vs payload model; errors before release (1.5 s .. release) and after (2.6 .. 4 s),
%     and ctc_payload_on must drop from 1 to 0 exactly at t_release_plan.
%
%   throwMat : baseline input MAT (q_cmd, qd_cmd, qdd_cmd, t_release_plan). Part 2 is skipped if empty.
%   Output   : results/payload_check.csv, results/payload_check.png
here = fileparts(mfilename('fullpath'));
if nargin < 1, throwMat = 'baseline_input_decel.mat'; end
p2pMat = fullfile(here,'p2p_input.mat');
assert(isfile(p2pMat), 'Run make_p2p_trajectory first (needs %s).', p2pMat);
assert(isfile(fullfile(here,'Gen3_J2_payload.slx')), 'Run build_stage3_payload first.');

% ---------------- part 1: slow point-to-point ----------------
load_system(fullfile(here,'Gen3_J2_payload.slx'));
mw = get_param('Gen3_J2_payload','ModelWorkspace');
rb012 = make_payload_robot(mw.getVariable('robotToTest'), 0.12);
r = { run_p2p_ideal(p2pMat, 'Gen3_J2_slot',    'ideal'), ...
      run_p2p_ideal(p2pMat, 'Gen3_J2_payload', 'payload'), ...
      run_p2p_ideal(p2pMat, 'Gen3_J2_payload', 'payload_0p12', struct('robotPayload', rb012)) };
rows = {};
for k = 1:3
    hw = r{k}.t >= 3.6 & r{k}.t <= 4.4;           % tilted hold window
    eh = mean(r{k}.e(hw,:), 1);  em = max(abs(r{k}.e), [], 1);
    rows(end+1,:) = {'p2p', r{k}.tag, 1e3*em(2), 1e3*em(4), 1e3*em(6), 1e3*max(em), ...
                     1e3*eh(2), 1e3*eh(4), NaN, NaN}; %#ok<AGROW>
end

% ---------------- part 2: throw ----------------
thr = {};
if ~isempty(throwMat) && (isfile(throwMat) || ~isempty(which(throwMat)))
    S = load(throwMat); tr = S.t_release_plan;
    for mdl = {'Gen3_J2_slot', 'Gen3_J2_payload'}
        m = mdl{1}; if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
        in = Simulink.SimulationInput(m);
        for v = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'}, in = in.setVariable(v{1}, S.(v{1})); end
        out = sim(in);
        t = out.q_actual.Time; q = as_rows(out.q_actual.Data);
        [tu, iu] = unique(S.q_cmd.Time, 'last'); Q = as_rows(S.q_cmd.Data);
        e = interp1(tu, Q(iu,:), t, 'linear', 'extrap') - q;
        pre = t >= 1.5 & t < tr; post = t >= 2.6;
        sw = NaN;
        if strcmp(m,'Gen3_J2_payload')
            f = out.ctc_payload_on; fv = f.Data(:); ft = f.Time;
            assert(all(fv(ft < tr - 1e-9) == 1) && all(fv(ft > tr + 1e-9) == 0), ...
                'ctc_payload_on does not switch at t_release_plan = %g s', tr);
            sw = ft(find(fv == 0, 1));             % first logged time with the payload off
        end
        rows(end+1,:) = {'throw', m, 1e3*max(abs(e(pre,2))), 1e3*max(abs(e(pre,4))), ...
            1e3*max(abs(e(pre,6))), 1e3*max(abs(e(pre,:)),[],'all'), ...
            1e3*max(abs(e(post,2))), 1e3*max(abs(e(post,4))), sw, tr}; %#ok<AGROW>
        thr(end+1,:) = {m, t, e}; %#ok<AGROW>
    end
else
    fprintf('Part 2 (throw) skipped: %s not found.\n', throwMat);
end

T = cell2table(rows, 'VariableNames', {'Motion','Case','J2_maxErr_mrad','J4_maxErr_mrad', ...
    'J6_maxErr_mrad','AllJoints_maxErr_mrad','J2_holdOrPostErr_mrad','J4_holdOrPostErr_mrad', ...
    'PayloadOff_at_s','t_release_s'});
disp(T);
rd = fullfile(here,'results'); if ~exist(rd,'dir'), mkdir(rd); end
writetable(T, fullfile(rd,'payload_check.csv'));

% ---------------- figure ----------------
fg = figure('Visible','off','Position',[50 50 1000 750]); tiledlayout(2,1);
nexttile; hold on;
for k = 1:3, plot(r{k}.t, 1e3*r{k}.e(:,2)); end
for k = 1:3, plot(r{k}.t, 1e3*r{k}.e(:,4), '--'); end
ylabel('error (mrad)'); grid on; title('Slow motion: joint 2 (solid) and joint 4 (dashed) tracking error');
legend('j2 no payload','j2 payload 0.1 kg','j2 payload set 0.12 kg', ...
       'j4 no payload','j4 payload 0.1 kg','j4 payload set 0.12 kg','Location','eastoutside');
nexttile; hold on;
if ~isempty(thr)
    for k = 1:size(thr,1), plot(thr{k,2}, 1e3*thr{k,3}(:,2)); end
    xline(tr, 'k:', 'release');
    legend('j2 no payload (slot)', 'j2 payload-aware CTC', 'Location','eastoutside');
end
ylabel('error (mrad)'); xlabel('time (s)'); grid on; title('Throw: joint 2 tracking error');
exportgraphics(fg, fullfile(rd,'payload_check.png'), 'Resolution',130); close(fg);

R = struct('table', T);
ok = rows{2,6} < 0.5;
assert(ok, 'Payload-aware CTC still leaves %.3g mrad error on the slow motion.', rows{2,6});
fprintf('PASS: payload-aware CTC removes the hold error (max %.3g mrad on all joints).\n', rows{2,6});
end

function d = as_rows(d)
if ndims(d) == 3, d = reshape(permute(d,[3 1 2]), size(d,3), []); end
if size(d,2) ~= 7 && size(d,1) == 7, d = d.'; end
end

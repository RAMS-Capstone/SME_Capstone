function cmp = test_stage3_slot(inputMat, srcfile)
% TEST_STAGE3_SLOT  The slot must change nothing while J2_Actuator is a pass-through.
%   Runs the original Gen3 throw baseline and Gen3_J2_slot.slx with the same reference
%   trajectory and compares q, qd (7 joints) and the joint torques.
%
%   inputMat : the baseline input MAT with q_cmd, qd_cmd, qdd_cmd, t_release_plan
%              (default 'baseline_input_decel.mat' on the MATLAB path; it is on Minhyuk's PC)
%   srcfile  : (optional) full path of Gen3_throw_baseline.slx, found automatically otherwise
%   Pass     : max |difference| < 1e-9 for every signal.
%   Output   : stage3_robot/results/stage3_slot_check.csv, stage3_slot_check.png
if nargin < 1, inputMat = 'baseline_input_decel.mat'; end
if nargin < 2, srcfile = ''; end
here = fileparts(mfilename('fullpath'));
S = load(inputMat);
need = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'};
miss = need(~isfield(S,need));
assert(isempty(miss), 'Input MAT is missing: %s', strjoin(miss,', '));

orig = 'Gen3_throw_baseline'; slot = 'Gen3_J2_slot';
for mdl = {orig, slot}, if bdIsLoaded(mdl{1}), close_system(mdl{1},0); end, end
load_system(find_baseline(here, srcfile));
load_system(fullfile(here,[slot '.slx']));
out = struct();
for mdl = {orig, slot}
    m = mdl{1}; in = Simulink.SimulationInput(m);
    fn = fieldnames(S);
    for k = 1:numel(fn), in = in.setVariable(fn{k}, S.(fn{k})); end
    out.(m) = sim(in);
end
o = out.(orig); n = out.(slot);

% same solver + same arithmetic -> time vectors should match; interpolate only if they do not
[tq, qo, qn]   = common(o.q_actual,  n.q_actual);
[~,  vo, vn]   = common(o.qd_actual, n.qd_actual);
[tt, to, tn]   = common(o.tau_joint, n.tau_joint);
[~,  tc, ta]   = common(n.tau_joint, n.tau_applied);
cmp = struct( ...
  'same_time_grid',      isequal(o.q_actual.Time, n.q_actual.Time), ...
  'q_maxdiff_rad',       max(abs(qo - qn), [], 'all'), ...
  'qd_maxdiff_rad_s',    max(abs(vo - vn), [], 'all'), ...
  'tau_cmd_maxdiff_Nm',  max(abs(to - tn), [], 'all'), ...
  'tau_applied_vs_cmd_Nm', max(abs(tc - ta), [], 'all'), ...
  'j2_q_maxdiff_rad',    max(abs(qo(:,2) - qn(:,2))));
disp(cmp);
rd = fullfile(here,'results'); if ~exist(rd,'dir'), mkdir(rd); end
writetable(struct2table(cmp), fullfile(rd,'stage3_slot_check.csv'));

f = figure('Visible','off','Position',[50 50 1000 600]); tiledlayout(2,1);
nexttile; plot(tq, qo(:,2), 'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(tq, qn(:,2), '--');
ylabel('q_2 (rad)'); legend('original','with slot'); grid on;
title('Joint 2: original baseline vs model with actuator slot (pass-through)');
nexttile; plot(tt, to(:,2), 'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(tt, ta(:,2), '--');
ylabel('\tau_2 (N m)'); xlabel('time (s)'); legend('original command','applied via slot'); grid on;
exportgraphics(f, fullfile(rd,'stage3_slot_check.png'), 'Resolution',130); close(f);

tol = 1e-9;
ok = cmp.q_maxdiff_rad < tol && cmp.qd_maxdiff_rad_s < tol && ...
     cmp.tau_cmd_maxdiff_Nm < tol && cmp.tau_applied_vs_cmd_Nm < tol;
close_system(orig,0); close_system(slot,0);
assert(ok, 'The slot changed the result. Check the Selector/Assignment index and the CTC->plant wiring.');
fprintf('PASS: actuator slot is transparent (all differences < %g).\n', tol);
end

function [t, a, b] = common(x, y)
% Returns both signals as [N x 7] on one time grid.
t = x.Time; a = squeeze2(x.Data);
if isequal(x.Time, y.Time)
    b = squeeze2(y.Data);
else
    b = interp1(y.Time, squeeze2(y.Data), t, 'linear', 'extrap');
end
end

function d = squeeze2(d)
% timeseries data may be [N x 7] or [7 x 1 x N]; return [N x 7]
if ndims(d) == 3, d = permute(d, [3 1 2]); d = reshape(d, size(d,1), []); end
if size(d,2) ~= 7 && size(d,1) == 7, d = d.'; end
end

function f = find_baseline(here, given)
% Where is Gen3_throw_baseline.slx? 1) argument  2) repo layout  3) same folder  4) MATLAB path
name = 'Gen3_throw_baseline.slx';
cands = {given, fullfile(here,'..','models','original',name), fullfile(here,name), which(name)};
f = '';
for k = 1:numel(cands)
    c = cands{k};
    if ~isempty(c) && isfile(c), f = c; return; end
end
error(['Gen3_throw_baseline.slx not found. Pass its full path as an argument, ' ...
       'e.g. build_stage3_slot(''C:/SME_Capstone/models/original/Gen3_throw_baseline.slx'')']);
end

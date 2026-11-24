function T = time_check_actuator(stopTime, cfgIdx)
% TIME_CHECK_ACTUATOR  Which solver setting makes Gen3_J2_act fast enough, and is it still accurate?
%   Runs the actuator model (ctc_Jr on, aligned start) on the slow point-to-point input for a short
%   time with several solver settings. Prints wall-clock time, solver steps, the estimate for the
%   full 8.5 s run, and the difference from the first (tightest) setting.
%
%   stopTime : simulated seconds per run (default 0.8 = 0.5 s hold + first 0.3 s of the move)
%   cfgIdx   : which settings to try (default all), e.g. [2 3] to skip the slow reference
%   Settings:
%     1 ode23t  MaxStep 1e-4  RelTol 1e-5   (build_stage3_actuator default, reference)
%     2 ode23t  MaxStep 1e-3  RelTol 1e-4   (implicit, 10x larger step: 25 steps per gear period)
%     3 ode45   MaxStep 2e-4  RelTol 1e-5   (explicit: no Jacobians; stable for the 1 kHz loop)
%     4 ode23t  MaxStep 1e-3  RelTol 1e-4   in accelerator mode (first run compiles, ~1 min)
%   Pick one for the full run, e.g.
%     run_stage3_actuator(struct('solver', struct('SolverName','ode45','MaxStep','2e-4')))
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(stopTime), stopTime = 0.8; end
cfg = { struct('SolverName','ode23t','MaxStep','1e-4','RelTol','1e-5'), 'ode23t 1e-4';
        struct('SolverName','ode23t','MaxStep','1e-3','RelTol','1e-4'), 'ode23t 1e-3';
        struct('SolverName','ode45', 'MaxStep','2e-4','RelTol','1e-5'), 'ode45 2e-4';
        struct('SolverName','ode23t','MaxStep','1e-3','RelTol','1e-4','SimulationMode','accelerator'), ...
                                                                       'ode23t 1e-3 accel' };
if nargin < 2 || isempty(cfgIdx), cfgIdx = 1:size(cfg,1); end
S = load(fullfile(here,'p2p_input.mat'));
m = 'Gen3_J2_act';
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
act = j2_actuator_init(m, S);

rows = {}; ref = [];
for k = cfgIdx
    in = Simulink.SimulationInput(m);
    for v = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'}, in = in.setVariable(v{1}, S.(v{1})); end
    in = in.setVariable('act', act, 'Workspace', m);
    in = in.setModelParameter('StopTime', num2str(stopTime), 'OutputOption','RefineOutputTimes', 'Refine','1');
    f = fieldnames(cfg{k,1});
    for i = 1:numel(f), in = in.setModelParameter(f{i}, cfg{k,1}.(f{i})); end
    try, get_param(m,'SimMechanicsOpenEditorOnUpdate'); ...
         in = in.setModelParameter('SimMechanicsOpenEditorOnUpdate','off'); catch, end
    fprintf('[%d] %-18s %.2g s simulated ... ', k, cfg{k,2}, stopTime); tk = tic;
    out = sim(in);
    w = toc(tk);
    t = out.q_actual.Time; q = squeeze(out.q_actual.Data);
    if size(q,1) ~= numel(t), q = q.'; end
    ta = out.j2_act_log.Time; L = squeeze(out.j2_act_log.Data);
    if size(L,1) ~= numel(ta), L = L.'; end
    tg = (0:1e-3:stopTime)';
    q2 = rs(t, q(:,2), tg); tj = rs(ta, L(:,9), tg);
    if isempty(ref), ref = struct('q2', q2, 'tj', tj); end
    dq = 1e3*max(abs(q2 - ref.q2)); dtj = max(abs(tj - ref.tj));
    fprintf('%.1f s wall, %d steps, full 8.5 s ~ %.0f min, |dq2| %.2g mrad, |dtauJ| %.2g N m\n', ...
            w, numel(t), w*8.5/stopTime/60, dq, dtj);
    rows(end+1,:) = {cfg{k,2}, stopTime, w, numel(t), w*8.5/stopTime/60, dq, dtj}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', {'Setting','Simulated_s','Wall_s','Steps', ...
    'EstFull_min','DiffQ2_vs_first_mrad','DiffTauJ_vs_first_Nm'});
disp(T);
fprintf(['Differences are against the first setting tried. act_jr and act_nojr are expected to differ by\n' ...
         '~1 mrad on joint 2: a setting is good enough if DiffQ2 stays below ~0.01 mrad.\n']);
end

function y = rs(t, d, tq)
[tu, iu] = unique(t, 'last');
y = interp1(tu, d(iu,:), tq, 'linear', 'extrap');
end

function T = time_check_actuator(stopTime, cfgIdx)
% TIME_CHECK_ACTUATOR  Where does the time go in Gen3_J2_act? Short runs, one line per setting.
%
%   The actuator needs small solver steps (1 kHz current loop, 40 Hz gear mode). If one solver
%   step of the Gen3 model is expensive (Multibody + 6 rigidBodyTree blocks in the CTC), many
%   small steps make a run take hours. This splits the two effects:
%     wall time per step  (cost of one step: compare ideal vs actuator at the same step size)
%     steps per second    (set by the solver settings)
%   and extrapolates to the full 8.5 s motion (the moving part needs at least as many steps).
%
%   stopTime : simulated seconds per run (default 0.05: a few hundred steps, seconds to minutes)
%   cfgIdx   : which settings to run (default all)
%     1 ideal  Gen3_J2_ctc  its own solver (what took 164 s for 8.5 s)
%     2 ideal  Gen3_J2_ctc  ode45 MaxStep 2e-4      (same step as 3, no actuator)
%     3 act    Gen3_J2_act  ode45 MaxStep 2e-4
%     4 act    Gen3_J2_act  ode23t MaxStep 1e-3 RelTol 1e-4
%     5 act    Gen3_J2_act  ode45 MaxStep 2e-4, accelerator mode (run twice: 1st compiles)
%   The wall time is printed as soon as each run finishes, so a setting that hangs is visible.
%   Pick the setting for the full run, e.g.
%     run_stage3_actuator(struct('solver', struct('SolverName','ode45','MaxStep','2e-4', ...
%                                                 'SimulationMode','accelerator')))
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(stopTime), stopTime = 0.05; end
o45 = struct('SolverName','ode45','MaxStep','2e-4','RelTol','1e-5');
cfg = { 'Gen3_J2_ctc', struct(),                                                   'ideal, own solver';
        'Gen3_J2_ctc', o45,                                                        'ideal, ode45 2e-4';
        'Gen3_J2_act', o45,                                                        'act, ode45 2e-4';
        'Gen3_J2_act', struct('SolverName','ode23t','MaxStep','1e-3','RelTol','1e-4'), 'act, ode23t 1e-3';
        'Gen3_J2_act', setfield(o45,'SimulationMode','accelerator'),               'act, ode45 2e-4 accel' }; %#ok<SFLD>
if nargin < 2 || isempty(cfgIdx), cfgIdx = 1:size(cfg,1); end
ensure_stage3_model('Gen3_J2_act');      % builds Gen3_J2_slot -> ... -> Gen3_J2_act if missing
S = load(ensure_p2p_input());

rows = {};
for k = cfgIdx
    m = cfg{k,1};
    if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
    nrep = 1 + strcmp(getfield_or(cfg{k,2},'SimulationMode','normal'), 'accelerator');
    for rep = 1:nrep
        in = Simulink.SimulationInput(m);
        for v = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'}, in = in.setVariable(v{1}, S.(v{1})); end
        if strcmp(m,'Gen3_J2_act'), in = in.setVariable('act', j2_actuator_init(m, S), 'Workspace', m); end
        in = in.setModelParameter('StopTime', num2str(stopTime), 'OutputOption','RefineOutputTimes', 'Refine','1');
        f = fieldnames(cfg{k,2});
        for i = 1:numel(f), in = in.setModelParameter(f{i}, cfg{k,2}.(f{i})); end
        try, get_param(m,'SimMechanicsOpenEditorOnUpdate'); ...
             in = in.setModelParameter('SimMechanicsOpenEditorOnUpdate','off'); catch, end
        lbl = cfg{k,3}; if nrep == 2, lbl = sprintf('%s (run %d)', lbl, rep); end
        fprintf('[%d] %-28s %.3g s simulated ... ', k, lbl, stopTime); tk = tic;
        out = sim(in);
        w = toc(tk); n = numel(out.q_actual.Time);
        est = w*8.5/stopTime/60;
        fprintf('%.1f s wall, %d steps, %.1f ms/step -> full 8.5 s ~ %.0f min\n', w, n, 1e3*w/n, est);
        rows(end+1,:) = {lbl, stopTime, w, n, 1e3*w/n, est}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'Setting','Simulated_s','Wall_s','Steps','ms_per_step','EstFull_min'});
disp(T);
fprintf(['Read it like this: ms/step of 2 vs 3 = cost of the actuator blocks; steps of 1 vs 2 = cost of the\n' ...
         'small step. The estimate assumes the hold at the start costs as much as the motion (lower bound).\n']);
end

function v = getfield_or(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

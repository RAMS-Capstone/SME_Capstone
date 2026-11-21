function T = time_check_payload(stopTime)
% TIME_CHECK_PAYLOAD  How slow is the payload model compared with the slot model?
%   Runs both models on the slow point-to-point input for a short time, without animation,
%   and prints wall-clock time, solver steps and steps per second.
%   stopTime : simulated seconds per run (default 1)
here = fileparts(mfilename('fullpath'));
if nargin < 1, stopTime = 1; end
S = load(fullfile(here,'p2p_input.mat'));
mods = {'Gen3_J2_slot','Gen3_J2_payload'};
rows = cell(2,5);
for k = 1:2
    m = mods{k};
    if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
    in = Simulink.SimulationInput(m);
    for v = {'q_cmd','qd_cmd','qdd_cmd','t_release_plan'}, in = in.setVariable(v{1}, S.(v{1})); end
    in = in.setModelParameter('StopTime', num2str(stopTime));
    try, get_param(m,'SimMechanicsOpenEditorOnUpdate'); ...
         in = in.setModelParameter('SimMechanicsOpenEditorOnUpdate','off'); catch, end
    fprintf('%s: %g s simulated ... ', m, stopTime); tk = tic;
    out = sim(in);
    w = toc(tk);
    n = numel(out.q_actual.Time);
    fprintf('%.1f s wall clock, %d logged steps\n', w, n);
    rows(k,:) = {m, stopTime, w, n, n/w};
end
T = cell2table(rows, 'VariableNames', {'Model','Simulated_s','Wall_s','Steps','StepsPerWall_s'});
disp(T);
fprintf('payload / slot wall-clock ratio: %.1f x,  step-count ratio: %.1f x\n', ...
    T.Wall_s(2)/T.Wall_s(1), T.Steps(2)/T.Steps(1));
fprintf('Estimated full 8.5 s slow motion: slot %.0f s, payload %.0f s\n', ...
    T.Wall_s(1)*8.5/stopTime, T.Wall_s(2)*8.5/stopTime);
end

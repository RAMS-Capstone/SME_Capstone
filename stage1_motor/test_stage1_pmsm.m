function results = test_stage1_pmsm
% TEST_STAGE1_PMSM  Runs the 5 stage-1 tests on J2_PMSM_Stage1.slx and compares them with the
% independent Python reference (verify_stage1.py: dq model == abc model to 2.5e-6 A).
%   T1 locked rotor, iq step 10 A          -> 1 kHz loop: 10-90% rise 0.35 ms, id = 0
%   T2 2583 rpm (baseline peak), iq 10 A   -> same rise, |v| stays below 12.47 V
%   T3 speed pushed to 470 rad/s 4-12 ms   -> voltage limit active, anti-windup recovery
%   T4 free rotor, +10 A then -10 A        -> acceleration Te/J, voltage-limited top speed
%   T5 iq command 30 A                     -> command limit 20.9 A
% Run build_stage1_pmsm first. Output: stage1_test_results.csv, stage1_tests.png
m = 'J2_PMSM_Stage1'; load_system(m);
p = j2_params();
ref = struct( ...                       % Python reference values (stage1_verification.json)
  'T1', struct('rise_ms',0.350,'vmag_max',3.864), ...
  'T2', struct('rise_ms',0.350,'vmag_max',11.731), ...
  'T3', struct('iq_min',-97.00,'recover_ms',2.38), ...
  'T4', struct('wm_20ms',109.73,'wm_peak',429.10), ...
  'T5', struct('iq_final',20.9));
S = { % name, T, iq_cmd [t;v], wm [t;v], free
 'T1', 6e-3,  [0 1e-3 6e-3; 0 10 10],   [0 6e-3; 0 0],                       0;
 'T2', 6e-3,  [0 1e-3 6e-3; 0 10 10],   [0 6e-3; 270.5 270.5],               0;
 'T3', 20e-3, [0 1e-3 20e-3; 0 10 10],  [0 4e-3 12e-3 20e-3; 270.5 470 270.5 270.5], 0;
 'T4', 0.3,   [0 1e-3 0.15 0.3; 0 10 -10 -10], [0 0.3; 0 0],                1;
 'T5', 4e-3,  [0 1e-3 4e-3; 0 30 30],   [0 4e-3; 0 0],                       0 };
rows = {}; f = figure('Visible','off','Position',[50 50 1100 1100]); tiledlayout(5,2);
for k = 1:size(S,1)
    [name,T,ic,wv,free] = S{k,:};
    u1 = setinterpmethod(timeseries(ic(2,:)',ic(1,:)'),'zoh');
    u2 = setinterpmethod(timeseries(wv(2,:)',wv(1,:)'),'zoh');
    ds = Simulink.SimulationData.Dataset; ds = ds.addElement(u1,'iq_cmd'); ds = ds.addElement(u2,'wm_imposed');
    in = Simulink.SimulationInput(m); in = in.setExternalInput(ds);
    in = in.setVariable('free_rotor',free,'Workspace',m);
    in = in.setModelParameter('StopTime',num2str(T));
    out = sim(in);
    t = out.stage1_log.Time; d = out.stage1_log.Data;
    id = d(:,1); iq = d(:,2); wm = d(:,4); vq = d(:,6); iqr = d(:,8); vqr = d(:,10); vmag = d(:,11);
    % ---- generic checks (every test) ----
    assert(all(isfinite(d),'all'), [name ': non-finite']);
    assert(max(vmag) <= p.Vmax*(1+1e-9), [name ': |v| above limit']);
    assert(max(abs(iqr)) <= p.Imax*(1+1e-12), [name ': iq command limit violated']);
    r = struct('test',name,'metric','','simulink',NaN,'python',NaN,'pass',true);
    switch name
        case {'T1','T2'}
            rise = (tcross(t,iq,9) - tcross(t,iq,1))*1e3;
            assert(max(abs(id)) < 1e-6, [name ': id not regulated']);
            r = add(r,'rise_10_90_ms',rise,ref.(name).rise_ms,0.03);
            r(end+1) = add1(name,'vmag_max_V',max(vmag),ref.(name).vmag_max,0.05);
        case 'T3'
            assert(any(vqr - vq > 1e-3), 'T3: voltage limit never active');
            rec = (tcross(t(t>12e-3), iq(t>12e-3), 9.8) - 12e-3)*1e3;
            assert(max(iq(t>12e-3)) < 10.05, 'T3: overshoot after recovery (windup)');
            r = add(r,'iq_min_A',min(iq),ref.T3.iq_min,2.0);
            r(end+1) = add1(name,'recover_to_9p8A_ms',rec,ref.T3.recover_ms,0.25);
        case 'T4'
            r = add(r,'wm_at_20ms',interp1(t,wm,0.02),ref.T4.wm_20ms,0.6);
            r(end+1) = add1(name,'wm_peak',max(wm),ref.T4.wm_peak,2.0);
            assert(interp1(t,wm,0.02) > 0, 'T4: positive iq must give positive speed');
        case 'T5'
            r = add(r,'iq_final_A',iq(end),ref.T5.iq_final,0.01);
    end
    for j = 1:numel(r), rows(end+1,:) = {r(j).test,r(j).metric,r(j).simulink,r(j).python,r(j).pass}; end %#ok<AGROW>
    nexttile; plot(t*1e3,iq,t*1e3,id,t*1e3,iqr,'k:'); grid on; ylabel('A'); title([name ' currents']);
    nexttile; plot(t*1e3,vmag,t*1e3,hypot(d(:,9),vqr),':'); yline(p.Vmax,'r'); grid on; ylabel('V'); title([name ' |v|']);
end
xlabel('time (ms)');
results = cell2table(rows,'VariableNames',{'Test','Metric','Simulink','Python','Pass'});
disp(results); writetable(results,'stage1_test_results.csv');
exportgraphics(f,'stage1_tests.png','Resolution',130); close(f);
assert(all(results.Pass), 'Some stage-1 metric differs from the Python reference');
fprintf('PASS: stage-1 hand-built PMSM + FOC matches the independent reference.\n');
end

function tc = tcross(t,y,level)
k = find(y >= level, 1);
assert(~isempty(k) && k > 1, 'level %.3g never crossed', level);
tc = t(k-1) + (level - y(k-1))*(t(k)-t(k-1))/(y(k)-y(k-1));
end
function r = add(r,metric,v,refv,tol)
r.metric = metric; r.simulink = v; r.python = refv; r.pass = abs(v-refv) <= tol;
end
function r = add1(name,metric,v,refv,tol)
r = struct('test',name,'metric',metric,'simulink',v,'python',refv,'pass',abs(v-refv) <= tol);
end

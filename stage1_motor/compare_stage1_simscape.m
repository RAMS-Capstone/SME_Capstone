function cmp = compare_stage1_simscape
% COMPARE_STAGE1_SIMSCAPE  Same iq command, same controller, two plants:
%   hand-built dq model (J2_PMSM_Stage1, free rotor)  vs  Simscape PMSM block (J2_PMSM_SimscapeRef).
% Pass = the hand-built motor reproduces the library motor. Output: stage1_vs_simscape.png/.csv
% Run build_stage1_pmsm and build_stage1_simscape_ref first.
T = 0.3; ic = [0 1e-3 0.15 T; 0 10 -10 -10];
u = setinterpmethod(timeseries(ic(2,:)',ic(1,:)'),'zoh');
z = setinterpmethod(timeseries([0;0],[0;T]),'zoh');

m1 = 'J2_PMSM_Stage1'; load_system(m1);
ds = Simulink.SimulationData.Dataset; ds = ds.addElement(u,'iq_cmd'); ds = ds.addElement(z,'wm_imposed');
in = Simulink.SimulationInput(m1); in = in.setExternalInput(ds);
in = in.setVariable('free_rotor',1,'Workspace',m1); in = in.setModelParameter('StopTime',num2str(T));
o1 = sim(in); t1 = o1.stage1_log.Time; d1 = o1.stage1_log.Data;   % id iq Te wm vd vq ...

m2 = 'J2_PMSM_SimscapeRef'; load_system(m2);
ds2 = Simulink.SimulationData.Dataset; ds2 = ds2.addElement(u,'iq_cmd');
in2 = Simulink.SimulationInput(m2); in2 = in2.setExternalInput(ds2);
in2 = in2.setModelParameter('StopTime',num2str(T));
o2 = sim(in2); t2 = o2.ref_log.Time; d2 = o2.ref_log.Data;        % id iq wm vd vq theta

tc = unique([t1; t2]); tc = tc(tc <= T);
I = @(t,y) interp1(t,y,tc,'linear');
dif = @(a,b) max(abs(a-b));
cmp = struct();
cmp.id_maxdiff_A  = dif(I(t1,d1(:,1)), I(t2,d2(:,1)));
cmp.iq_maxdiff_A  = dif(I(t1,d1(:,2)), I(t2,d2(:,2)));
cmp.wm_maxdiff    = dif(I(t1,d1(:,4)), I(t2,d2(:,3)));
cmp.wm_peak_dq    = max(d1(:,4)); cmp.wm_peak_simscape = max(d2(:,3));
cmp.vq_maxdiff_V  = dif(I(t1,d1(:,6)), I(t2,d2(:,5)));
disp(cmp); writetable(struct2table(cmp),'stage1_vs_simscape.csv');

f = figure('Visible','off','Position',[50 50 1000 700]); tiledlayout(3,1);
nexttile; plot(t2,d2(:,2),'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(t1,d1(:,2),'--',t1,d1(:,1));
legend('iq Simscape','iq hand-built','id hand-built'); ylabel('A'); grid on;
title('Same FOC controller, hand-built dq motor vs Simscape PMSM block');
nexttile; plot(t2,d2(:,3),'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(t1,d1(:,4),'--');
legend('\omega_m Simscape','\omega_m hand-built'); ylabel('rad/s'); grid on;
nexttile; plot(t2,d2(:,5),'LineWidth',3,'Color',[.6 .8 1]); hold on; plot(t1,d1(:,6),'--');
legend('vq Simscape','vq hand-built'); ylabel('V'); xlabel('time (s)'); grid on;
exportgraphics(f,'stage1_vs_simscape.png','Resolution',130); close(f);

ok = cmp.iq_maxdiff_A < 0.2 && cmp.id_maxdiff_A < 0.2 && cmp.wm_maxdiff < 0.01*cmp.wm_peak_dq;
if ok
    fprintf('PASS: hand-built motor matches the Simscape PMSM block.\n');
else
    warning(['Mismatch. First check the rotor-angle convention (axes_param d-axis vs q-axis) ' ...
             'and that the Simulink-PS converters are not filtering.']);
end
close_system(m1,0); close_system(m2,0);
end

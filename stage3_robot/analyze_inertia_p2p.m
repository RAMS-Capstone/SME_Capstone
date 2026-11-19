function R = analyze_inertia_p2p(outMat)
% ANALYZE_INERTIA_P2P  How much does the inertia seen by joint 2 change during the motion?
%   (stage 3, item 3 - the professor's point: "inertia between joints changes while moving")
%
%   Uses the joint angles logged by run_p2p_ideal and the robot model (robotToTest) to compute
%   the joint-space mass matrix M(q) along the motion, then compares
%     M22(t)            inertia of the arm about joint 2 (changes with q2, q3, q4, ...)
%     M23, M24          coupling: how much joint 3/4 acceleration loads joint 2
%     Jr = N^2*(Jm+J_WG) = 0.249 kg m^2   reflected rotor + wave-generator inertia (constant)
%   With a 100:1 gear, Jr is added to M22, which lowers the RELATIVE change the motor sees.
%   Result 2026-11-20: Jr is only ~16-20% of M22 for joint 2, so it hides little (13% -> 11%).
%
%   outMat : results/p2p_ideal_out.mat (default)
%   Output : results/p2p_inertia.csv, results/p2p_inertia.png
here = fileparts(mfilename('fullpath'));
if nargin < 1, outMat = fullfile(here,'results','p2p_ideal_out.mat'); end
L = load(outMat);
m = 'Gen3_J2_slot';
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
mw = get_param(m,'ModelWorkspace');
robot = copy(mw.getVariable('robotToTest'));   % copy: do not touch the model's object
robot.DataFormat = 'row';

% reflected inertia of the joint-2 actuator (values from stage1_motor/j2_params.m)
N = 100; Jm = 170e-7; JWG = 0.079e-4; Jr = N^2*(Jm + JWG);

% evaluate every 10 ms
[tu, iu] = unique(L.t, 'last'); qu = L.q(iu,:);
ts = (tu(1):0.01:tu(end))'; qs = interp1(tu, qu, ts);
n = numel(ts); M22 = zeros(n,1); M23 = M22; M24 = M22; M44 = M22;
for k = 1:n
    Mk = massMatrix(robot, qs(k,:));
    M22(k) = Mk(2,2); M23(k) = Mk(2,3); M24(k) = Mk(2,4); M44(k) = Mk(4,4);
end

pct = @(x) 100*(max(x) - min(x))/mean(x);
R = struct( ...
  'M22_min', min(M22), 'M22_max', max(M22), 'M22_change_pct', pct(M22), ...
  'M24_absmax', max(abs(M24)), 'M24_over_M22_pct', 100*max(abs(M24)./M22), ...
  'M23_absmax', max(abs(M23)), ...
  'Jr_reflected', Jr, ...
  'M22_plus_Jr_change_pct', pct(M22 + Jr), ...
  'Jr_share_of_total_pct_min', 100*min(Jr./(M22 + Jr)));
disp(R);
rd = fullfile(here,'results'); if ~exist(rd,'dir'), mkdir(rd); end
writetable(struct2table(R), fullfile(rd,'p2p_inertia.csv'));
writetable(table(ts, qs(:,2), qs(:,4), M22, M23, M24, M44, M22 + Jr, 'VariableNames', ...
    {'Time_s','q2_rad','q4_rad','M22','M23','M24','M44','M22_plus_Jr'}), ...
    fullfile(rd,'p2p_inertia_timeseries.csv'));

f = figure('Visible','off','Position',[50 50 1000 750]); tiledlayout(3,1);
nexttile; plot(ts, qs(:,2), ts, qs(:,4)); ylabel('q (rad)'); legend('q_2','q_4'); grid on;
title('Inertia seen by joint 2 during the slow point-to-point motion');
nexttile; plot(ts, M22, 'LineWidth',1.5); hold on; plot(ts, M22 + Jr, 'LineWidth',1.5);
yline(Jr, 'k:', sprintf('J_r = N^2(J_m+J_{WG}) = %.3f', Jr), 'LabelHorizontalAlignment','left');
ylabel('kg m^2'); legend(sprintf('M_{22} (arm only), change %.0f%%', R.M22_change_pct), ...
    sprintf('M_{22} + J_r (motor side), change %.0f%%', R.M22_plus_Jr_change_pct), 'Location','east'); grid on;
nexttile; plot(ts, M23, ts, M24); yline(0,'k:'); ylabel('kg m^2'); xlabel('time (s)');
legend('M_{23} coupling', 'M_{24} coupling'); grid on;
exportgraphics(f, fullfile(rd,'p2p_inertia.png'), 'Resolution',130); close(f);
fprintf(['M22 changes %.0f%% over the motion; with the reflected rotor inertia (%.3f kg m^2) ' ...
         'the motor sees a %.0f%% change.\n'], R.M22_change_pct, Jr, R.M22_plus_Jr_change_pct);
end

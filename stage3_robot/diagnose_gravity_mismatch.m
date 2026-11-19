function D = diagnose_gravity_mismatch(outMat)
% DIAGNOSE_GRAVITY_MISMATCH  Why do joints keep a steady error in the tilted hold, even with ideal torque?
%
%   Hypothesis: the CTC computes gravity with robotToTest, which has no ball, while the Multibody
%   plant carries a 0.1 kg ball welded to EndEffector_Link (Subsystem/Inertia).
%
%   Test (no new simulation needed, uses results/p2p_ideal_out.mat from run_p2p_ideal):
%     In a still hold (qd = qdd_ref = 0) the plant needs  tau = g_plant(q).
%     The CTC gives  tau = g_model(q) + M(q)*Kp*e.
%     So  delta = tau_applied - g_model(q)  is the gravity the model is missing,
%     and it must also equal  M(q)*Kp*e  (consistency check).
%     If delta == gravity torque of a 0.1 kg point mass at the end effector -> ball confirmed.
%
%   Also prints robot vs plant link masses and both gravity vectors, to rule out other causes.
%   Output: results/gravity_mismatch.csv
here = fileparts(mfilename('fullpath'));
if nargin < 1, outMat = fullfile(here,'results','p2p_ideal_out.mat'); end
L = load(outMat);
m = 'Gen3_J2_slot';
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
mw = get_param(m,'ModelWorkspace');
robot = copy(mw.getVariable('robotToTest')); robot.DataFormat = 'row';

% ---------- 1. gravity vectors and link masses ----------
fprintf('robotToTest.Gravity = %s   (plant MechanismConfiguration: [0 0 -9.81])\n', mat2str(robot.Gravity));
blks = find_system([m '/Subsystem'], 'LookUnderMasks','all', 'FollowLinks','on', 'Name','Inertia');
fprintf('\n%-22s %12s %12s\n', 'body', 'robot [kg]', 'plant [kg]');
plantMass = containers.Map;
for k = 1:numel(blks)
    parentName = get_param(get_param(blks{k},'Parent'),'Name');
    if strcmp(get_param(blks{k},'Parent'), [m '/Subsystem']), parentName = 'BALL (Subsystem/Inertia)'; end
    plantMass(parentName) = str2double(get_param(blks{k},'Mass'));
end
mr = 0; mp = 0;
for k = 1:robot.NumBodies
    b = robot.Bodies{k}; pmv = NaN;
    if isKey(plantMass, b.Name), pmv = plantMass(b.Name); mp = mp + pmv; end
    mr = mr + b.Mass;
    fprintf('%-22s %12.4f %12.4f\n', b.Name, b.Mass, pmv);
end
if isKey(plantMass,'BALL (Subsystem/Inertia)')
    fprintf('%-22s %12s %12.4f\n', 'ball', '-', plantMass('BALL (Subsystem/Inertia)'));
end
fprintf('%-22s %12.4f %12.4f   (links matched by name)\n\n', 'TOTAL', mr, mp);

% ---------- 2. steady holds: tilted (q0+delta) and upright (q0) ----------
kpText = get_param([m '/Computed Torque Controller/Kp'],'Gain');
Kp = str2num(kpText); %#ok<ST2NM>
if isempty(Kp)                                   % gain given as a variable name
    try, Kp = mw.getVariable(kpText); catch, Kp = evalin('base', kpText); end
end
fprintf('CTC Kp gain: %s\n', kpText);
if isscalar(Kp), Kp = Kp*eye(7); elseif isvector(Kp), Kp = diag(Kp); end
win = {'tilted hold 3.6-4.4 s', [3.6 4.4]; 'upright hold 0.1-0.4 s', [0.1 0.4]};
% robot + 0.1 kg ball at the end-effector origin (same as the plant's weld, no offset)
rb = copy(robot);
ball = rigidBody('ball'); ball.Mass = 0.1; ball.CenterOfMass = [0 0 0];
ball.Inertia = [1.6e-5 1.6e-5 1.6e-5 0 0 0];
ball.Joint = rigidBodyJoint('ball_weld','fixed');
eeName = 'EndEffector_Link'; if ~any(strcmp(robot.BodyNames, eeName)), eeName = robot.BodyNames{end}; end
addBody(rb, ball, eeName);

rows = {};
for w = 1:size(win,1)
    sel = L.t >= win{w,2}(1) & L.t <= win{w,2}(2);
    q = mean(L.q(sel,:),1); tau = mean(L.tau(sel,:),1); e = mean(L.qr(sel,:) - L.q(sel,:),1);
    g_model = gravityTorque(robot, q);
    g_ball  = gravityTorque(rb, q) - g_model;
    delta   = tau - g_model;
    via_err = (massMatrix(robot, q) * Kp * e.').';
    fprintf('== %s  (max |qd| = %.2g rad/s)\n', win{w,1}, max(abs(L.qd(sel,:)),[],'all'));
    fprintf(' joint | tau_applied | g_model | delta=tau-g | M*Kp*e | ball gravity | delta-ball\n');
    for j = 1:7
        fprintf('   %d   | %11.4f | %7.4f | %10.4f | %6.4f | %12.4f | %10.4f\n', ...
            j, tau(j), g_model(j), delta(j), via_err(j), g_ball(j), delta(j) - g_ball(j));
        rows(end+1,:) = {win{w,1}, j, tau(j), g_model(j), delta(j), via_err(j), g_ball(j), delta(j)-g_ball(j)}; %#ok<AGROW>
    end
end
T = cell2table(rows, 'VariableNames', {'Window','Joint','TauApplied_Nm','GravityModel_Nm', ...
    'Delta_Nm','MKpE_Nm','BallGravity_Nm','DeltaMinusBall_Nm'});
rd = fullfile(here,'results'); if ~exist(rd,'dir'), mkdir(rd); end
writetable(T, fullfile(rd,'gravity_mismatch.csv'));

% ---------- 3. verdict (tilted hold, all joints) ----------
% A joint passes if what is left after removing the ball is < 10% of the mismatch OR < 0.01 N m.
% (The absolute floor matters: joint 4 carries only ~0.002 N m, where small dynamic terms of the
%  slightly-moving hold, |qd| ~ 0.01 rad/s, are of the same size.)
tl = T(strcmp(T.Window, win{1,1}), :);
res = abs(tl.DeltaMinusBall_Nm); tolj = max(0.1*abs(tl.Delta_Nm), 0.01);
D = struct('table', T, 'residual_Nm', res.', 'tolerance_Nm', tolj.');
if all(res <= tolj)
    fprintf(['\nCONFIRMED: the missing gravity is the 0.1 kg ball. Joint 2: mismatch %.4f N m, ' ...
             'ball %.4f N m, residual %.4f N m.\n'], tl.Delta_Nm(2), tl.BallGravity_Nm(2), tl.DeltaMinusBall_Nm(2));
else
    fprintf(['\nNOT (only) the ball on joint(s) %s. ' ...
             'Check the link mass table and the gravity vectors above.\n'], mat2str(find(res > tolj).'));
end
end

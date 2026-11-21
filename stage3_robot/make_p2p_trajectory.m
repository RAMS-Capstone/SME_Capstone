function traj = make_p2p_trajectory(opts)
% MAKE_P2P_TRAJECTORY  Slow point-to-point motion inside the Gen3 speed limit (stage 3, item 2).
%
%   Motion (rest-to-rest 5th-order polynomial on every segment):
%     hold q0 -> move to q0+delta -> hold -> move back to q0 -> hold
%   Default delta moves joint 2 (shoulder) +0.8 rad and joint 4 (elbow) -0.8 rad, so gravity
%   load and the coupling between the two joints change during the motion. Other joints hold.
%
%   opts fields (all optional)
%     delta   [7x1] rad   joint displacement           default [0 0.8 0 -0.8 0 0 0]'
%     T_move  s           duration of each move        default 3
%     t_hold  [1x3] s     hold before/between/after    default [0.5 1 1]
%     vmax    [7x1] rad/s speed limit to check         default 1.36 (Gen3 large actuator, 13 RPM)
%     dt      s           sample time of the command   default 1e-3
%     model   char        model whose q0/dq0/robot are used  default 'Gen3_J2_slot'
%     outfile char        MAT written for the model    default 'p2p_input.mat' next to this file
%
%   Output MAT holds q_cmd, qd_cmd, qdd_cmd (timeseries, 7x1xN like the baseline input),
%   t_release_plan (= 1e6, the ball is never released) and p2p_t_end (stop time).
%   Peak speed of a 5th-order rest-to-rest move = 1.875*|delta|/T_move.
here = fileparts(mfilename('fullpath'));
if nargin < 1, opts = struct(); end
d = struct('delta',[0 0.8 0 -0.8 0 0 0]', 'T_move',3, 't_hold',[0.5 1 1], ...
           'vmax',1.36*ones(7,1), 'dt',1e-3, 'model','Gen3_J2_slot', ...
           'outfile',fullfile(here,'p2p_input.mat'));
fn = fieldnames(d);
for k = 1:numel(fn), if ~isfield(opts,fn{k}), opts.(fn{k}) = d.(fn{k}); end, end
delta = opts.delta(:); vmax = opts.vmax(:);
if isscalar(vmax), vmax = vmax*ones(7,1); end

% ---- start state and joint limits from the model itself ----
m = opts.model;
if ~bdIsLoaded(m), load_system(fullfile(here,[m '.slx'])); end
mw = get_param(m,'ModelWorkspace');
q0 = mw.getVariable('q0'); q0 = q0(:);
dq0 = mw.getVariable('dq0'); dq0 = dq0(:);
robot = mw.getVariable('robotToTest');
if max(abs(dq0)) > 1e-6
    warning('dq0 is not zero (max %.3g rad/s). The trajectory starts from rest anyway.', max(abs(dq0)));
end
lim = joint_limits(robot);

% ---- segments ----
T = opts.T_move; th = opts.t_hold; dt = opts.dt;
q1 = q0 + delta;
tb = cumsum([0, th(1), T, th(2), T, th(3)]);      % segment boundaries
t_end = tb(end);
t = (0:dt:t_end)';
if t(end) < t_end, t(end+1) = t_end; end
N = numel(t); Q = zeros(N,7); QD = Q; QDD = Q;
for i = 1:N
    ti = t(i);
    if ti < tb(2)
        Q(i,:) = q0';
    elseif ti < tb(3)
        [s, sd, sdd] = quintic((ti - tb(2))/T);
        Q(i,:) = (q0 + delta*s)'; QD(i,:) = (delta*sd/T)'; QDD(i,:) = (delta*sdd/T^2)';
    elseif ti < tb(4)
        Q(i,:) = q1';
    elseif ti < tb(5)
        [s, sd, sdd] = quintic((ti - tb(4))/T);
        Q(i,:) = (q1 - delta*s)'; QD(i,:) = (-delta*sd/T)'; QDD(i,:) = (-delta*sdd/T^2)';
    else
        Q(i,:) = q0';
    end
end

% ---- checks ----
vpk = max(abs(QD))'; apk = max(abs(QDD))';
assert(all(vpk <= vmax + 1e-12), 'Speed limit exceeded on joint(s) %s', mat2str(find(vpk > vmax)'));
margin = deg2rad(5);
for k = 1:7
    if all(isfinite(lim(k,:)))
        assert(min(Q(:,k)) >= lim(k,1) + margin && max(Q(:,k)) <= lim(k,2) - margin, ...
            'Joint %d leaves its limits [%.2f %.2f] rad (5 deg margin)', k, lim(k,1), lim(k,2));
    end
end

% ---- save in the same format as the baseline input ----
q_cmd   = timeseries(permute(Q,  [2 3 1]), t, 'Name','q_cmd');
qd_cmd  = timeseries(permute(QD, [2 3 1]), t, 'Name','qd_cmd');
qdd_cmd = timeseries(permute(QDD,[2 3 1]), t, 'Name','qdd_cmd');
t_release_plan = 1e6;       %#ok<NASGU> ball stays welded (release step never happens)
p2p_t_end = t_end;          %#ok<NASGU>
save(opts.outfile, 'q_cmd','qd_cmd','qdd_cmd','t_release_plan','p2p_t_end');

traj = struct('t',t,'q',Q,'qd',QD,'qdd',QDD,'q0',q0,'q1',q1,'t_end',t_end, ...
              'v_peak',vpk,'a_peak',apk,'vmax',vmax,'limits',lim,'opts',opts);
fprintf('Saved %s  (t_end = %.2f s)\n', opts.outfile, t_end);
fprintf(' joint |  q0 [rad] | delta [rad] | peak speed [rad/s] | %% of limit | peak acc [rad/s^2]\n');
for k = 1:7
    fprintf('   %d   | %9.4f | %11.4f | %18.4f | %9.1f | %10.4f\n', ...
        k, q0(k), delta(k), vpk(k), 100*vpk(k)/vmax(k), apk(k));
end
end

function [s, sd, sdd] = quintic(x)
% rest-to-rest 5th-order blend on x in [0,1]: s(0)=0, s(1)=1, zero speed and acceleration at both ends
x = min(max(x,0),1);
s   = 10*x^3 - 15*x^4 + 6*x^5;
sd  = 30*x^2 - 60*x^3 + 30*x^4;
sdd = 60*x   - 180*x^2 + 120*x^3;
end

function lim = joint_limits(robot)
% position limits of the movable joints, in tree order (7x2, inf when unbounded)
lim = [];
for k = 1:robot.NumBodies
    j = robot.Bodies{k}.Joint;
    if ~strcmp(j.Type,'fixed'), lim(end+1,:) = j.PositionLimits; end %#ok<AGROW>
end
assert(size(lim,1) == 7, 'Expected 7 movable joints in robotToTest, found %d', size(lim,1));
lim(abs(lim) > 1e3) = sign(lim(abs(lim) > 1e3))*inf;
end

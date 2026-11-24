function [act, tau0] = j2_actuator_init(model, S, ctcJr)
% J2_ACTUATOR_INIT  Start the joint-2 actuator in equilibrium with the robot (motor-shaft alignment).
%
%   The gear spring is  tau_J = Kg*(s*theta_m/N - q_J) + cg*(s*omega_m/N - qd_J).
%   With theta_m0 = 0 the spring would start wound up by -Kg*q0(2): at q0(2) = 0.1 rad that is
%   1400 N m on a 70 N m gear. So the motor shaft starts where the joint is, PLUS the deflection
%   that already carries the first CTC torque command tau0 (otherwise the joint drops by tau0/Kg
%   and the 40 Hz gear mode rings at t = 0):
%       theta_m0 = s*N*(q0(2) + tau0(2)/Kg),   omega_m0 = s*N*dq0(2)   ->   tau_J(0) = tau0(2)
%
%   tau0 is the CTC output at t = 0, computed exactly like the CTC does it:
%       a0 = qdd_ref(0) + Kp*(q_ref(0) - q0) + Kd*(qd_ref(0) - dq0)
%       tau0 = inverseDynamics(robot, q0, dq0, a0) + ctc_Jr .* a0
%   robot = robotPayload while the ball is held at t = 0 (t_release_plan > 0), robotToTest otherwise.
%
%   model : loaded model that holds act, q0, dq0, robotPayload, robotToTest and the CTC Kp/Kd blocks
%   S     : input struct with q_cmd, qd_cmd, qdd_cmd (timeseries, or 7x1 values at t = 0) and
%           t_release_plan, e.g. load('p2p_input.mat')
%   ctcJr : 7x1 reflected inertia the CTC uses in this run (default: model value of ctc_Jr)
%   act   : copy of the model's act struct with theta_m0, omega_m0 set (pass it per run with
%           SimulationInput.setVariable('act', act, 'Workspace', model))
mw = get_param(model,'ModelWorkspace');
act = mw.getVariable('act');
q0 = mw.getVariable('q0'); q0 = q0(:);
dq0 = mw.getVariable('dq0'); dq0 = dq0(:);
if nargin < 3 || isempty(ctcJr), ctcJr = mw.getVariable('ctc_Jr'); end
ctcJr = ctcJr(:);
if S.t_release_plan > 0, rb = mw.getVariable('robotPayload'); else, rb = mw.getVariable('robotToTest'); end
rb = copy(rb); rb.DataFormat = 'column';

c = [model '/Computed Torque Controller'];
Kp = str2double(get_param([c '/Kp'],'Gain'));
Kd = str2double(get_param([c '/Kd'],'Gain'));
a0 = first(S.qdd_cmd) + Kp*(first(S.q_cmd) - q0) + Kd*(first(S.qd_cmd) - dq0);
tau0 = inverseDynamics(rb, q0, dq0, a0) + ctcJr.*a0;

act.theta_m0 = act.s*act.N*(q0(2) + tau0(2)/act.Kg);
act.omega_m0 = act.s*act.N*dq0(2);
end

function v = first(ts)
% first sample of a 7x1xN (or Nx7) timeseries as a 7x1 column (a plain 7x1 vector is used as is)
if isnumeric(ts), v = ts(:); return; end
d = ts.Data;
if ndims(d) == 3, v = d(:,:,1); else, v = d(1,:); end
v = v(:);
end

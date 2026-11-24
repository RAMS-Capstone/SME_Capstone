function build_stage3_actuator(actuatorModel, solver)
% BUILD_STAGE3_ACTUATOR  Puts the stage-2 joint-2 actuator into the Gen3 slot  ->  Gen3_J2_act.slx
%
%   Gen3_J2_ctc.slx (slot + payload CTC + ctc_Jr option) is copied, and the contents of
%   J2_Actuator_Slot/J2_Actuator (ideal pass-through until now) are replaced by the contents of
%   stage2_actuator/J2_Actuator.slx - the same copyContentsToSubSystem step the stage-2 test bench
%   uses, so what was tested on the 1-DOF bench is what runs in the robot:
%       tau_cmd_J -> torque_to_iq -> FOC (+-20.9 A, |v| <= 12.47 V) -> dq PMSM -> motor shaft
%                 -> harmonic-drive spring Kg, cg -> tau_J  (q_J, qd_J come back from the robot)
%
%   What else changes in the copy (Gen3_J2_ctc.slx itself is not touched):
%   1  model workspace "act" = J2_Actuator's act. Its theta_m0/omega_m0 are aligned with the robot's
%      start pose here (j2_actuator_init), so the gear spring starts carrying the first CTC torque
%      instead of -Kg*q0(2). Runs with another input recompute it (run_stage3_actuator does).
%   2  ctc_Jr = [0 Jr 0 0 0 0 0]' (on): the CTC knows the rotor it now drives (design decision 2).
%   3  solver ode23t, MaxStep 1e-3 s, RelTol 1e-4 (stiff: 1 kHz current loop, 40 Hz gear mode,
%      10 rad/s CTC loop). Fastest setting in time_check_actuator (2026-11-24); run_stage3_actuator
%      checks it against a 5x smaller step. The ideal models keep their own solver settings.
%   4  speed: the robotics blocks run as 'Code generation' instead of 'Interpreted execution'
%      (set_robot_blocks, same numbers, needs a C compiler), and the root Inverse Kinematics chain,
%      which only feeds a Scope, is commented out. With interpreted blocks and the small step a
%      run took hours (2026-11-24).
%
%   actuatorModel : (optional) full path of J2_Actuator.slx. Default ../stage2_actuator/J2_Actuator.slx;
%                   built with build_stage2_actuator if it does not exist yet.
%   solver        : (optional) struct overriding the solver settings, e.g. struct('MaxStep','5e-5').
%                   Field RobotBlocks = 'Interpreted execution' keeps the blocks interpreted
%                   (when no C compiler is installed).
here = fileparts(mfilename('fullpath'));
src = 'Gen3_J2_ctc'; m = 'Gen3_J2_act';
srcfile = fullfile(here,[src '.slx']);
assert(isfile(srcfile), 'Run build_stage3_ctc_jr first (needs %s).', srcfile);
s2 = fullfile(here,'..','stage2_actuator');
if nargin < 1 || isempty(actuatorModel), actuatorModel = fullfile(s2,'J2_Actuator.slx'); end
if nargin < 2, solver = struct(); end
if ~isfile(actuatorModel)
    assert(isfile(fullfile(s2,'build_stage2_actuator.m')), ...
        'J2_Actuator.slx not found. Pass its full path: build_stage3_actuator(''C:/.../J2_Actuator.slx'')');
    fprintf('J2_Actuator.slx not found -> building it with stage2_actuator/build_stage2_actuator\n');
    addpath(s2); build_stage2_actuator();
    actuatorModel = fullfile(s2,'J2_Actuator.slx');
end
[~, an] = fileparts(actuatorModel);
for b = {src, m, an}, if bdIsLoaded(b{1}), close_system(b{1},0); end, end
load_system(srcfile);
save_system(src, fullfile(here,[m '.slx']));        % loaded copy is now Gen3_J2_act
load_system(actuatorModel);
amw = get_param(an,'ModelWorkspace');
act = amw.getVariable('act');

% ---- 1. actuator into the slot (ports: tau_cmd_J, q_J, qd_J -> tau_J, unchanged) ----
slot = [m '/J2_Actuator_Slot']; a = [slot '/J2_Actuator'];
assert(getSimulinkBlockHandle(a) > 0, 'J2_Actuator_Slot/J2_Actuator not found in %s', src);
L = { 'Select_tau/1','J2_Actuator/1'; 'Select_q/1','J2_Actuator/2'; 'Select_qd/1','J2_Actuator/3';
      'J2_Actuator/1','Put_back_tau/2' };
for k = 1:size(L,1), delete_line(slot, L{k,1}, L{k,2}); end
Simulink.SubSystem.deleteContents(a);
Simulink.BlockDiagram.copyContentsToSubSystem(an, a);
close_system(an, 0);
for k = 1:size(L,1), add_line(slot, L{k,1}, L{k,2}, 'autorouting','on'); end
ip = numel(find_system(a,'SearchDepth',1,'BlockType','Inport'));
op = numel(find_system(a,'SearchDepth',1,'BlockType','Outport'));
assert(ip == 3 && op == 1, 'J2_Actuator must have 3 inputs and 1 output (found %d / %d)', ip, op);
set_param(a,'Description',['Joint-2 actuator (stage 2, copied from J2_Actuator.slx by build_stage3_actuator): ' ...
    'torque->iq, FOC, dq PMSM, motor shaft, harmonic-drive spring. Parameters: model workspace act.']);

% ---- 2. parameters: act (aligned start), CTC knows the rotor ----
mw = get_param(m,'ModelWorkspace');
mw.assignin('act', act);
mw.assignin('ctc_Jr', [0 act.Jr 0 0 0 0 0]');
q0 = mw.getVariable('q0');
S0 = struct('q_cmd', q0(:), 'qd_cmd', zeros(7,1), 'qdd_cmd', zeros(7,1), ...
            't_release_plan', 1e6);                     % holding still at q0 with the ball
[act, tau0] = j2_actuator_init(m, S0);
mw.assignin('act', act);
fprintf(['Start alignment at q0: q0(2) = %.4f rad, CTC holding torque tau0(2) = %.3f N m\n' ...
         '  theta_m0 = s*N*(q0(2) + tau0/Kg) = %.3f rad (motor side)\n' ...
         '  without it (theta_m0 = 0) the gear would start at %.0f N m\n'], ...
        q0(2), tau0(2), act.theta_m0, -act.Kg*q0(2));

% ---- 3. solver ----
sp = struct('SolverName','ode23t', 'MaxStep','1e-3', 'RelTol','1e-4', 'AbsTol','auto');
rbMode = 'Code generation';
f = fieldnames(solver); for k = 1:numel(f), sp.(f{k}) = solver.(f{k}); end
if isfield(sp,'RobotBlocks'), rbMode = sp.RobotBlocks; sp = rmfield(sp,'RobotBlocks'); end
f = fieldnames(sp);     for k = 1:numel(f), set_param(m, f{k}, sp.(f{k})); end

% ---- 4. speed: compiled robotics blocks, unused IK chain off ----
set_robot_blocks(m, rbMode);
ik = find_system(m, 'SearchDepth',1, 'RegExp','on', 'ReferenceBlock','Inverse Kinematics$');
for k = 1:numel(ik)
    % IK block, its inputs (Coordinate Transformation Conversion, Constant1, Delay) and their inputs
    chain = get_param(ik{k},'Handle');
    for lvl = 1:2
        src = [];
        for h = chain
            pc = get_param(h,'PortConnectivity');
            src = [src, pc.SrcBlock]; %#ok<AGROW>
        end
        chain = unique([chain, src(src > 0)]);
    end
    for h = chain, set_param(h, 'Commented','on'); end
    fprintf('Commented out the unused IK chain: %s\n', strjoin(get_param(chain,'Name'), ', '));
end

set_param(m,'Description', ['Gen3 + stage-2 joint-2 actuator in the slot (FOC, dq PMSM, rotor, elastic gear) ' ...
    '+ payload CTC with ctc_Jr on. Built by build_stage3_actuator.']);
save_system(m); close_system(m,0);
fprintf('Saved %s.slx (Jr = %.3f kg m^2 in the CTC, %s, MaxStep %s)\n', fullfile(here,m), act.Jr, ...
        sp.SolverName, sp.MaxStep);
end

function build_stage3_payload(payloadMass)
% BUILD_STAGE3_PAYLOAD  Gen3_J2_slot + payload-aware CTC  ->  Gen3_J2_payload.slx
%
%   Why: the plant holds a 0.1 kg ball but the CTC model (robotToTest) has none, so the arm keeps
%   a steady error in tilted holds (diagnose_gravity_mismatch.m). Real controllers have a payload
%   setting that the program switches when it grasps / releases - the controller itself commands
%   the release, so it knows when.
%
%   What changes, inside "Computed Torque Controller" only:
%     Gravity Torque, Joint Space Mass Matrix, Velocity Product Torque (robotToTest)
%       -> each gets a twin that uses robotPayload (= robotToTest + ball, make_payload_robot)
%       -> a Switch picks the twin while holding, the original after release
%     holding = (Clock < t_release_plan)  - the SAME parameter that opens the plant's weld joint
%     (Step at t_release_plan inside Subsystem). Logged as ctc_payload_on.
%
%   payloadMass : mass the controller assumes [kg], default 0.1 (= the real ball).
%                 For a wrong-setting study, pass another value or override robotPayload with
%                 SimulationInput.setVariable (see test_stage3_payload).
here = fileparts(mfilename('fullpath'));
if nargin < 1, payloadMass = 0.1; end
src = 'Gen3_J2_slot'; m = 'Gen3_J2_payload';
srcfile = fullfile(here,[src '.slx']);
assert(isfile(srcfile), 'Run build_stage3_slot first (needs %s).', srcfile);
for b = {src, m}, if bdIsLoaded(b{1}), close_system(b{1},0); end, end
load_system(srcfile);
save_system(src, fullfile(here,[m '.slx']));       % loaded copy is now Gen3_J2_payload

mw = get_param(m,'ModelWorkspace');
robot = mw.getVariable('robotToTest');
mw.assignin('payload_mass', payloadMass);
mw.assignin('robotPayload', make_payload_robot(robot, payloadMass));

c = [m '/Computed Torque Controller'];
assert(getSimulinkBlockHandle(c) > 0, 'Computed Torque Controller not found');

% ---- holding flag: t < t_release_plan ----
add_block('simulink/Sources/Clock',[c '/Clock']);
add_block('simulink/Sources/Constant',[c '/t_release'],'Value','t_release_plan');
add_block('simulink/Logic and Bit Operations/Relational Operator',[c '/holding'],'Operator','<');
add_line(c,'Clock/1','holding/1','autorouting','on');
add_line(c,'t_release/1','holding/2','autorouting','on');
add_block('simulink/Signal Attributes/Data Type Conversion',[c '/holding_double'],'OutDataTypeStr','double');
add_line(c,'holding/1','holding_double/1','autorouting','on');
add_block('simulink/Sinks/To Workspace',[c '/log_payload_on'], ...
    'VariableName','ctc_payload_on','SaveFormat','Timeseries');
add_line(c,'holding_double/1','log_payload_on/1','autorouting','on');

% ---- twin blocks + switches ----
%  {block, its inputs (source ports), its destination}
T = { 'Gravity Torque',           {'Measured Configuration/1'},                          'Add1/1',    'Switch_gravity';
      'Joint Space Mass Matrix',  {'Measured Configuration/1'},                          'Product/1', 'Switch_mass';
      'Velocity Product Torque',  {'Measured Configuration/1','Measured Velocities/1'},  'Add1/2',    'Switch_velprod' };
for k = 1:size(T,1)
    [b, ins, dst, sw] = T{k,:};
    twin = [b ' payload'];
    add_block([c '/' b], [c '/' twin]);
    set_param([c '/' twin], 'RigidBodyTree', 'robotPayload');
    for i = 1:numel(ins), add_line(c, ins{i}, sprintf('%s/%d', twin, i), 'autorouting','on'); end
    delete_line(c, [b '/1'], dst);
    add_block('simulink/Signal Routing/Switch', [c '/' sw], 'Criteria','u2 ~= 0');
    add_line(c, [twin '/1'], [sw '/1'], 'autorouting','on');     % true  (holding) -> with ball
    add_line(c, 'holding_double/1', [sw '/2'], 'autorouting','on');
    add_line(c, [b '/1'], [sw '/3'], 'autorouting','on');        % false (released) -> no ball
    add_line(c, [sw '/1'], dst, 'autorouting','on');
end
Simulink.BlockDiagram.arrangeSystem(c);

set_param(m,'Description',sprintf(['Gen3 throw + joint-2 slot + payload-aware CTC (assumed payload %.3g kg, ' ...
    'switched off at t_release_plan).'], payloadMass));
save_system(m); close_system(m,0);
fprintf('Saved %s.slx (CTC payload %.3g kg until t_release_plan)\n', fullfile(here,m), payloadMass);
end

function build_stage3_ctc_jr
% BUILD_STAGE3_CTC_JR  Gen3_J2_payload + reflected-inertia option in the CTC  ->  Gen3_J2_ctc.slx
%
%   Design decision 2 (2026-11-16): the CTC KNOWS the actuator. A geared motor adds the
%   reflected rotor inertia  Jr = N^2*(Jm + J_WG)  to its joint (0.249 kg m^2 for joint 2:
%   N = 100, Jm = 1.7e-5, J_WG = 0.79e-5, see stage1_motor/j2_params.m). The controller adds it
%   to the diagonal of its mass matrix:
%       tau = (M(q) + diag(ctc_Jr)) * a  +  C(q,qd)*qd  +  g(q),   a = qdd_ref + Kp*e + Kd*ed
%
%   ctc_Jr : 7x1 model-workspace variable, default zeros(7,1) = OFF (identical to Gen3_J2_payload).
%            Turn it on only when the actuator (with its rotor) is in the slot, e.g.
%            ctc_Jr = [0 0.249 0 0 0 0 0]'. Set it per run with SimulationInput.setVariable.
%
%   From now on Gen3_J2_ctc.slx is the stage-3 working model (slot + payload + Jr option).
here = fileparts(mfilename('fullpath'));
src = 'Gen3_J2_payload'; m = 'Gen3_J2_ctc';
srcfile = fullfile(here,[src '.slx']);
assert(isfile(srcfile), 'Run build_stage3_payload first (needs %s).', srcfile);
for b = {src, m}, if bdIsLoaded(b{1}), close_system(b{1},0); end, end
load_system(srcfile);
save_system(src, fullfile(here,[m '.slx']));

mw = get_param(m,'ModelWorkspace');
mw.assignin('ctc_Jr', zeros(7,1));

c = [m '/Computed Torque Controller'];
assert(getSimulinkBlockHandle([c '/Switch_mass']) > 0, ...
    'Switch_mass not found: Gen3_J2_payload must come from build_stage3_payload.');
delete_line(c, 'Switch_mass/1', 'Product/1');
add_block('simulink/Sources/Constant', [c '/Reflected_inertia'], 'Value','diag(ctc_Jr)');
add_block('simulink/Math Operations/Sum', [c '/M_plus_Jr'], 'Inputs','++');
add_line(c, 'Switch_mass/1', 'M_plus_Jr/1', 'autorouting','on');
add_line(c, 'Reflected_inertia/1', 'M_plus_Jr/2', 'autorouting','on');
add_line(c, 'M_plus_Jr/1', 'Product/1', 'autorouting','on');
Simulink.BlockDiagram.arrangeSystem(c);

set_param(m,'Description', ['Gen3 throw + joint-2 slot + payload-aware CTC + reflected-inertia option ' ...
    '(ctc_Jr, default 0 = off). Stage-3 working model.']);
save_system(m); close_system(m,0);
fprintf('Saved %s.slx (ctc_Jr = 0, option off)\n', fullfile(here,m));
end

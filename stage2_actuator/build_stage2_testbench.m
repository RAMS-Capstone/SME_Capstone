function build_stage2_testbench
% BUILD_STAGE2_TESTBENCH  1-DOF test bench for J2_Actuator.slx  ->  J2_Actuator_TestBench.slx
%
%   The actuator's contents are copied into subsystem J2_Actuator (same thing the Gen3 slot will do).
%   Load (pendulum, J_L ~ Gen3 M22):  J_L*qdd = tau_J - G*sin(q) - b*qd
%     bench.fixed = 1 -> joint held: q_J = qd_J = 0 and the load does not move
%     bench.fixed = 0 -> pendulum
%   A second, identical load is driven directly by tau_cmd_ideal (ideal torque) for comparison.
%   Inputs : In1 tau_cmd_act [N m] (to the actuator), In2 tau_cmd_ideal [N m] (to the ideal load)
%   Logs   : j2_act_log (inside the actuator), bench_log = [q qd q_ideal qd_ideal tau_cmd_act tau_cmd_ideal]
here = fileparts(mfilename('fullpath'));
src = 'J2_Actuator'; tb = 'J2_Actuator_TestBench';
assert(isfile(fullfile(here,[src '.slx'])), 'Run build_stage2_actuator first.');
for b = {src, tb}, if bdIsLoaded(b{1}), close_system(b{1},0); end, end
load_system(fullfile(here,[src '.slx']));
mws = get_param(src,'ModelWorkspace'); act = mws.getVariable('act');

new_system(tb);
mw = get_param(tb,'ModelWorkspace');
mw.assignin('act', act);
mw.assignin('bench', struct('JL',1.2, 'G',15, 'b',0.5, 'fixed',1));
set_param(tb,'Solver','ode23t','MaxStep','5e-5','RelTol','1e-6','AbsTol','1e-8','StopTime','1');

a = [tb '/J2_Actuator'];
add_block('simulink/Ports & Subsystems/Subsystem', a); Simulink.SubSystem.deleteContents(a);
Simulink.BlockDiagram.copyContentsToSubSystem(src, a);
close_system(src, 0);

add_block('simulink/Sources/In1',[tb '/tau_cmd_act'],'Port','1');
add_block('simulink/Sources/In1',[tb '/tau_cmd_ideal'],'Port','2');
add_block('simulink/Sources/Constant',[tb '/fixed'],'Value','bench.fixed');
add_block('simulink/Sources/Constant',[tb '/zero'],'Value','0');
add_block('simulink/Sources/Constant',[tb '/free_gate'],'Value','1-bench.fixed');
load_blocks(tb, 'act');      % pendulum driven by the actuator
load_blocks(tb, 'ideal');    % same pendulum driven by the ideal torque
add_block('simulink/Signal Routing/Switch',[tb '/q_to_act'],'Criteria','u2 ~= 0');
add_block('simulink/Signal Routing/Switch',[tb '/qd_to_act'],'Criteria','u2 ~= 0');
add_block('simulink/Signal Routing/Mux',[tb '/bench_mux'],'Inputs','6');
add_block('simulink/Sinks/To Workspace',[tb '/log_bench'],'VariableName','bench_log','SaveFormat','Timeseries');

E = { 'tau_cmd_act/1','J2_Actuator/1';
      'zero/1','q_to_act/1'; 'fixed/1','q_to_act/2'; 'q_act/1','q_to_act/3';
      'zero/1','qd_to_act/1'; 'fixed/1','qd_to_act/2'; 'qd_act/1','qd_to_act/3';
      'q_to_act/1','J2_Actuator/2'; 'qd_to_act/1','J2_Actuator/3';
      'J2_Actuator/1','sum_act/1'; 'free_gate/1','gate_act/2';
      'tau_cmd_ideal/1','sum_ideal/1'; 'free_gate/1','gate_ideal/2';
      'q_act/1','bench_mux/1'; 'qd_act/1','bench_mux/2'; 'q_ideal/1','bench_mux/3'; 'qd_ideal/1','bench_mux/4';
      'tau_cmd_act/1','bench_mux/5'; 'tau_cmd_ideal/1','bench_mux/6'; 'bench_mux/1','log_bench/1' };
for k = 1:size(E,1), add_line(tb, E{k,1}, E{k,2}, 'autorouting','on'); end
Simulink.BlockDiagram.arrangeSystem(tb);
save_system(tb, fullfile(here,[tb '.slx'])); close_system(tb, 0);
fprintf('Saved %s.slx\n', fullfile(here,tb));
end

function load_blocks(tb, tag)
% J_L*qdd = tau - G*sin(q) - b*qd, acceleration multiplied by (1 - fixed)
n = @(s) [s '_' tag];
add_block('simulink/Math Operations/Sum',            [tb '/' n('sum')],   'Inputs','+--');
add_block('simulink/Math Operations/Gain',           [tb '/' n('inv_JL')],'Gain','1/bench.JL');
add_block('simulink/Math Operations/Product',        [tb '/' n('gate')]);
add_block('simulink/Continuous/Integrator',          [tb '/' n('qd')],    'InitialCondition','0');
add_block('simulink/Continuous/Integrator',          [tb '/' n('q')],     'InitialCondition','0');
add_block('simulink/Math Operations/Trigonometric Function', [tb '/' n('sin')], 'Operator','sin');
add_block('simulink/Math Operations/Gain',           [tb '/' n('gravity')],'Gain','bench.G');
add_block('simulink/Math Operations/Gain',           [tb '/' n('damping')],'Gain','bench.b');
L = { n('sum'),'/1', n('inv_JL'),'/1';  n('inv_JL'),'/1', n('gate'),'/1';
      n('gate'),'/1', n('qd'),'/1';     n('qd'),'/1', n('q'),'/1';
      n('q'),'/1', n('sin'),'/1';       n('sin'),'/1', n('gravity'),'/1';
      n('gravity'),'/1', n('sum'),'/2'; n('qd'),'/1', n('damping'),'/1';
      n('damping'),'/1', n('sum'),'/3' };
for k = 1:size(L,1), add_line(tb, [L{k,1} L{k,2}], [L{k,3} L{k,4}], 'autorouting','on'); end
end

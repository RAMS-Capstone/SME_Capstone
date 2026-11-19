function build_stage3_slot
% BUILD_STAGE3_SLOT  Copies the Gen3 throw baseline and inserts a joint-2 "actuator slot".
%
%   Original  : CTC  --tau(7)-->  Subsystem (Gen3 Multibody plant)
%   New model : CTC  --tau_cmd(7)-->  J2_Actuator_Slot  --tau_applied(7)-->  Subsystem
%                                      ^ q(7), qd(7) from the plant
%
%   Inside the slot, joint 2 is taken out of the vector (Selector), passed through the
%   J2_Actuator subsystem, and put back (Assignment). The other 6 joints are untouched.
%   J2_Actuator is an IDEAL pass-through for now: tau_J = tau_cmd_J. Stage 2 replaces its
%   contents with motor + FOC + gear; its ports (tau_cmd_J, q_J, qd_J -> tau_J) stay the same.
%
%   The original models/original/Gen3_throw_baseline.slx is never saved over.
%   Output: stage3_robot/Gen3_J2_slot.slx
here    = fileparts(mfilename('fullpath'));
srcfile = fullfile(here,'..','models','original','Gen3_throw_baseline.slx');
dstfile = fullfile(here,'Gen3_J2_slot.slx');
src = 'Gen3_throw_baseline'; m = 'Gen3_J2_slot';
if bdIsLoaded(m), close_system(m,0); end
if bdIsLoaded(src), close_system(src,0); end
load_system(srcfile);
save_system(src, dstfile);            % the loaded copy is now named Gen3_J2_slot; source file untouched
J = 2;                                % joint index served by this slot

ctc = [m '/Computed Torque Controller']; plant = [m '/Subsystem'];
assert(getSimulinkBlockHandle(ctc) > 0 && getSimulinkBlockHandle(plant) > 0, ...
    'Expected blocks "Computed Torque Controller" and "Subsystem" at the model root.');

% ---- 1. cut the CTC -> plant branch (the CTC -> To Workspace tau_joint branch stays) ----
try
    delete_line(m,'Computed Torque Controller/1','Subsystem/1');
catch
    ph = get_param(plant,'PortHandles'); delete_line(get_param(ph.Inport(1),'Line'));
end

% ---- 2. slot subsystem ----
s = [m '/J2_Actuator_Slot'];
pp = get_param(plant,'Position');
add_block('simulink/Ports & Subsystems/Subsystem', s, ...
    'Position',[pp(1)-230, pp(2), pp(1)-90, pp(2)+90]);
Simulink.SubSystem.deleteContents(s);
set_param(s,'Description',sprintf(['Joint-%d actuator slot. Selector takes joint %d out of the 7-vector, ' ...
    'J2_Actuator computes the applied joint torque, Assignment puts it back.'],J,J));
add_block('simulink/Sources/In1',[s '/tau_cmd'],'Port','1');
add_block('simulink/Sources/In1',[s '/q'],'Port','2');
add_block('simulink/Sources/In1',[s '/qd'],'Port','3');
sel = {'Select_tau','Select_q','Select_qd'};
for k = 1:3
    add_block('simulink/Signal Routing/Selector',[s '/' sel{k}], 'NumberOfDimensions','1', ...
        'IndexMode','One-based','IndexOptions','Index vector (dialog)','Indices',num2str(J), ...
        'InputPortWidth','7');
end
add_block('simulink/Math Operations/Assignment',[s '/Put_back_tau'], 'NumberOfDimensions','1', ...
    'IndexMode','One-based','IndexOptions','Index vector (dialog)','Indices',num2str(J), ...
    'OutputInitialize','Initialize using input port <Y0>');
add_block('simulink/Sinks/Out1',[s '/tau_applied'],'Port','1');

% ---- 3. J2_Actuator: ideal pass-through for now ----
a = [s '/J2_Actuator'];
add_block('simulink/Ports & Subsystems/Subsystem', a); Simulink.SubSystem.deleteContents(a);
set_param(a,'Description',['IDEAL pass-through (tau_J = tau_cmd_J). Stage 2 replaces the contents with ' ...
    'torque->iq, FOC, dq PMSM, gear (N=100, s=-1, Kg) and rotor inertia. Keep the 3 inputs / 1 output.']);
add_block('simulink/Sources/In1',[a '/tau_cmd_J'],'Port','1');
add_block('simulink/Sources/In1',[a '/q_J'],'Port','2');
add_block('simulink/Sources/In1',[a '/qd_J'],'Port','3');
add_block('simulink/Sinks/Out1',[a '/tau_J'],'Port','1');
add_block('simulink/Sinks/Terminator',[a '/q_unused']);
add_block('simulink/Sinks/Terminator',[a '/qd_unused']);
add_line(a,'tau_cmd_J/1','tau_J/1'); add_line(a,'q_J/1','q_unused/1'); add_line(a,'qd_J/1','qd_unused/1');
Simulink.BlockDiagram.arrangeSystem(a);

E = { 'tau_cmd/1','Select_tau/1'; 'q/1','Select_q/1'; 'qd/1','Select_qd/1';
      'Select_tau/1','J2_Actuator/1'; 'Select_q/1','J2_Actuator/2'; 'Select_qd/1','J2_Actuator/3';
      'tau_cmd/1','Put_back_tau/1'; 'J2_Actuator/1','Put_back_tau/2'; 'Put_back_tau/1','tau_applied/1' };
for k = 1:size(E,1), add_line(s,E{k,1},E{k,2},'autorouting','on'); end
Simulink.BlockDiagram.arrangeSystem(s);

% ---- 4. root wiring + logging of the torque that actually reaches the robot ----
add_line(m,'Computed Torque Controller/1','J2_Actuator_Slot/1','autorouting','on');
add_line(m,'Subsystem/1','J2_Actuator_Slot/2','autorouting','on');
add_line(m,'Subsystem/2','J2_Actuator_Slot/3','autorouting','on');
add_line(m,'J2_Actuator_Slot/1','Subsystem/1','autorouting','on');
add_block('simulink/Sinks/To Workspace',[m '/To Workspace tau_applied'], ...
    'VariableName','tau_applied','SaveFormat','Timeseries', ...
    'Position',[pp(1)-120, pp(4)+40, pp(1)-40, pp(4)+70]);
add_line(m,'J2_Actuator_Slot/1','To Workspace tau_applied/1','autorouting','on');

set_param(m,'Description',['Gen3 throw baseline + joint-2 actuator slot (stage 3 prep). ' ...
    'tau_joint = CTC command, tau_applied = torque that reaches the joints.']);
save_system(m);
close_system(m,0);
fprintf('Saved %s (joint-%d slot, ideal pass-through)\n', dstfile, J);
end

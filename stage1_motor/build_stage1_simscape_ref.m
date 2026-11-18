function build_stage1_simscape_ref
% BUILD_STAGE1_SIMSCAPE_REF  Creates J2_PMSM_SimscapeRef.slx: the SAME FOC controller
% (copied from J2_PMSM_Stage1) driving the Simscape library PMSM block with the SAME
% parameters, through three-phase voltage sources. Used only as a validation reference
% for the hand-built model (this is where Dong-ha's Simscape setup is reused).
%
% Port indices of the Simscape blocks were read from Dong-ha's PMSM_FOC_.slx wiring:
%   Controlled Voltage Source : LConn1 = +,  RConn1 = vT (PS input), RConn2 = -
%   Current Sensor            : LConn1 = +,  RConn1 = I (PS output), RConn2 = -
%   PMSM (expanded ports)     : LConn1..4 = a,b,c,n ; RConn1 = R (rotor), RConn2 = C (case)
%   Ideal Rot. Motion Sensor  : LConn1 = R ; RConn1 = C, RConn2 = W, RConn3 = A
%   Simulink-PS: RConn1 out ;  PS-Simulink: LConn1 in ; Solver Config: RConn1
src = 'J2_PMSM_Stage1'; m = 'J2_PMSM_SimscapeRef';
load_system(src); p = j2_params();
if bdIsLoaded(m), close_system(m,0); end
new_system(m); w = get_param(m,'ModelWorkspace'); w.assignin('p',p);
set_param(m,'Solver','ode23t','MaxStep','1e-5','RelTol','1e-6','AbsTol','1e-8','StopTime','0.3');

% ---------- Simulink side ----------
add_block('simulink/Sources/In1',[m '/iq_cmd'],'Interpolate','off');
add_block([src '/FOC_current_ctrl'],[m '/FOC_current_ctrl']);          % identical controller
add_block('simulink/Math Operations/Gain',[m '/we_angle'],'Gain','p.pp');   % theta_e = pp*theta_m
add_block('simulink/Signal Routing/Mux',[m '/vdq_theta'],'Inputs','3');
phase = {'a','+0'; 'b','-2*pi/3'; 'c','+2*pi/3'};
for k = 1:3   % inverse Park + inverse Clarke (amplitude invariant), d-axis aligned with phase a
    add_block('simulink/User-Defined Functions/Fcn',[m '/v' phase{k,1}], ...
        'Expr',sprintf('u(1)*cos(u(3)%s) - u(2)*sin(u(3)%s)',phase{k,2},phase{k,2}));
end
add_block('simulink/Signal Routing/Mux',[m '/iabc_theta'],'Inputs','4');
add_block('simulink/User-Defined Functions/Fcn',[m '/park_d'],'Expr', ...
  '2/3*(u(1)*cos(u(4)) + u(2)*cos(u(4)-2*pi/3) + u(3)*cos(u(4)+2*pi/3))');
add_block('simulink/User-Defined Functions/Fcn',[m '/park_q'],'Expr', ...
  '-2/3*(u(1)*sin(u(4)) + u(2)*sin(u(4)-2*pi/3) + u(3)*sin(u(4)+2*pi/3))');
add_block('simulink/Signal Routing/Mux',[m '/log_mux'],'Inputs','6');
add_block('simulink/Sinks/To Workspace',[m '/ref_log'],'VariableName','ref_log','SaveFormat','Timeseries');

% ---------- Simscape side ----------
pm = [m '/PMSM'];
add_block('ee_lib/Electromechanical/Permanent Magnet/PMSM', pm);
set_param(pm,'port_option','ee.enum.threePhasePort.expanded');
set_param(pm,'winding_type','ee.enum.electromech.statorconfiguration.wye', ...
  'machine_param','ee.enum.pmsm.parameterization.constant', ...
  'pmflux_param','ee.enum.pmflux.fluxlinkage','stator_param','ee.enum.statorparam.LdLqL0', ...
  'axes_param','ee.enum.rotorangle.daxis','loss_param','ee.enum.ironloss.none');
set_param(pm,'nPolePairs','p.pp','pm_flux_linkage','p.psi','Ld','p.Ld','Lq','p.Lq', ...
  'L0','p.Ld','Rs','p.Rs','J','p.J_test','lam','0');
add_block('nesl_utility/Solver Configuration',[m '/Solver Configuration']);
add_block('fl_lib/Electrical/Electrical Elements/Electrical Reference',[m '/GND']);
add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference',[m '/MechRef']);
add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor',[m '/RotorSensor']);
for k = 1:3
    x = phase{k,1};
    add_block('fl_lib/Electrical/Electrical Sources/Controlled Voltage Source',[m '/V_' x]);
    add_block('fl_lib/Electrical/Electrical Sensors/Current Sensor',[m '/I_' x]);
    add_block('nesl_utility/Simulink-PS Converter',[m '/S2PS_' x],'Unit','V');
    nodelay([m '/S2PS_' x]);
    add_block('nesl_utility/PS-Simulink Converter',[m '/PS2S_i' x],'Unit','A');
end
add_block('nesl_utility/PS-Simulink Converter',[m '/PS2S_w'],'Unit','rad/s');
add_block('nesl_utility/PS-Simulink Converter',[m '/PS2S_theta'],'Unit','rad');

H = @(b) get_param([m '/' b],'PortHandles');
pc = @(a,b) add_line(m,a,b,'autorouting','on');
g = H('GND'); P = H('PMSM'); R = H('RotorSensor'); M = H('MechRef');
pc(H('Solver Configuration').RConn(1), g.LConn(1));
pc(P.LConn(4), g.LConn(1));                                    % neutral n
for k = 1:3
    x = phase{k,1}; V = H(['V_' x]); I = H(['I_' x]);
    pc(V.LConn(1), I.LConn(1));                                 % source + -> sensor +
    pc(I.RConn(2), P.LConn(k));                                 % sensor - -> phase k
    pc(V.RConn(2), g.LConn(1));                                 % source - -> ground
    pc(H(['S2PS_' x]).RConn(1), V.RConn(1));                    % voltage command
    pc(I.RConn(1), H(['PS2S_i' x]).LConn(1));                   % measured current
end
pc(P.RConn(1), R.LConn(1)); pc(R.RConn(1), M.LConn(1)); pc(P.RConn(2), M.LConn(1));
pc(R.RConn(2), H('PS2S_w').LConn(1)); pc(R.RConn(3), H('PS2S_theta').LConn(1));

% ---------- Simulink wiring ----------
E = { 'iq_cmd/1','FOC_current_ctrl/1'; 'park_d/1','FOC_current_ctrl/2'; 'park_q/1','FOC_current_ctrl/3';
      'PS2S_w/1','FOC_current_ctrl/4'; 'PS2S_theta/1','we_angle/1';
      'FOC_current_ctrl/1','vdq_theta/1'; 'FOC_current_ctrl/2','vdq_theta/2'; 'we_angle/1','vdq_theta/3';
      'vdq_theta/1','va/1'; 'vdq_theta/1','vb/1'; 'vdq_theta/1','vc/1';
      'va/1','S2PS_a/1'; 'vb/1','S2PS_b/1'; 'vc/1','S2PS_c/1';
      'PS2S_ia/1','iabc_theta/1'; 'PS2S_ib/1','iabc_theta/2'; 'PS2S_ic/1','iabc_theta/3'; 'we_angle/1','iabc_theta/4';
      'iabc_theta/1','park_d/1'; 'iabc_theta/1','park_q/1';
      'park_d/1','log_mux/1'; 'park_q/1','log_mux/2'; 'PS2S_w/1','log_mux/3';
      'FOC_current_ctrl/1','log_mux/4'; 'FOC_current_ctrl/2','log_mux/5'; 'PS2S_theta/1','log_mux/6';
      'log_mux/1','ref_log/1' };
for k = 1:size(E,1), add_line(m,E{k,1},E{k,2},'autorouting','on'); end
add_block('simulink/Sinks/Terminator',[m '/diag_unused']);
add_line(m,'FOC_current_ctrl/3','diag_unused/1');
% LOG_COLUMNS: 1 id 2 iq 3 wm 4 vd 5 vq 6 theta_m
Simulink.BlockDiagram.arrangeSystem(m);
set_param(m,'SimulationCommand','update'); save_system(m);
close_system(src,0);
fprintf('Saved %s.slx (Simscape reference)\n',m);
end

function nodelay(b)
% Simulink-PS input must not be filtered (default 1 ms filter would distort a 0.16 ms current loop)
try
    set_param(b,'FilteringAndDerivatives','zero');
catch
    set_param(b,'FilteringAndDerivatives','filter','InputFilterTimeConstant','1e-7');
    warning('S2PS %s: zero-derivative option not found, using 0.1 us filter instead', b);
end
end

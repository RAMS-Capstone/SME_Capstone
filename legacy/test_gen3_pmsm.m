function test_gen3_pmsm
% Independent tests only; this function never loads the Gen3 robot model.
load('pmsm_validation_parameters.mat','p');
m='Gen3_J2_PMSM_Actuator'; load_system(m);
names={'zero','positive','negative','positive_limit','negative_limit','moving_positive','moving_negative','reversal','voltage_recovery','opposite_coordinate'};
allruns=struct; rows=cell(numel(names),7);
for k=1:numel(names)
 name=names{k}; t=[0;.05;.3;.6;1]; tau=zeros(5,1); vel=zeros(5,1); ps=p;
 switch name
  case 'positive',tau(2:end)=0.2;
  case 'negative',tau(2:end)=-0.2;
  case 'positive_limit',tau(2:end)=100;
  case 'negative_limit',tau(2:end)=-100;
  case 'moving_positive',tau(2:end)=0.2;vel(:)=1;
  case 'moving_negative',tau(2:end)=-0.2;vel(:)=-1;
  case 'reversal',tau=[0;.2;-.2;.2;.2];
  case 'voltage_recovery',tau(2:end)=0.2;vel=[0;3;3;0;0];
  case 'opposite_coordinate',tau(2:end)=.2;vel(:)=1;ps.s=-1;
 end
 u1=timeseries(tau,t);u1=setinterpmethod(u1,'zoh');
 u2=timeseries(vel,t);u2=setinterpmethod(u2,'zoh');
 ds=Simulink.SimulationData.Dataset;ds=ds.addElement(u1,'tau_cmd_J');ds=ds.addElement(u2,'omega_J');
 in=Simulink.SimulationInput(m);in=in.setExternalInput(ds);
 in=in.setVariable('p',ps,'Workspace',m);
 out=sim(in);d=out.diagnostic_log.Data;tt=out.diagnostic_log.Time;y=out.torque_log.Data;
 assert(all(isfinite(d),'all') && all(isfinite(y)),'Nonfinite result');
 assert(max(abs(d(:,3)))<=p.Imax+1e-9,'Current command limit violated');
 assert(max(abs(d(:,7)))<=p.Vqmax+1e-9,'q voltage limit violated');
 assert(max(abs(d(:,1)))<1e-5,'d current regulation failed');
 assert(max(abs(y-ps.s*p.N*d(:,4)))<1e-9,'Torque coordinate map failed');
 assert(max(abs(d(:,4).*d(:,5)-y.*(d(:,5)/(ps.s*p.N))))<1e-9,'Gear power conservation failed');
 target=max(-p.N*p.Kt*p.Imax,min(p.N*p.Kt*p.Imax,tau(end)));
 tail=tt>.95;err=max(abs(y(tail)-target));assert(err<2e-4,'Steady torque error');
 if ismember(name,{'positive','negative','positive_limit','negative_limit','reversal'})
  window=tt>.08 & tt<.29;
  firstTarget=max(-p.N*p.Kt*p.Imax,min(p.N*p.Kt*p.Imax,tau(2)));
  assert(max(abs(y(window)-firstTarget))<2e-4,'30 ms torque settling failed');
 end
 if strcmp(name,'voltage_recovery')
  assert(any(abs(d(:,8)-d(:,7))>.01),'Voltage saturation was not exercised');
  assert(max(abs(d(:,9)))<5,'Integrator windup');
 end
 rows(k,:)={name,err,max(abs(d(:,2))),max(abs(d(:,7))),max(abs(d(:,8))),max(abs(d(:,9))),true};
 allruns.(name)=out;
end
results=cell2table(rows,'VariableNames',{'Case','FinalTorqueError_Nm','PeakIq_A','PeakVq_V','PeakVqRaw_V','PeakPIqState_V','Passed'});
disp(results); writetable(results,'pmsm_test_results.csv');
h='Gen3_J2_PMSM_Standalone';out=sim(h);allruns.mechanics=out;
t=out.speed_log.Time;v=out.speed_log.Data;y=out.torque_log.Data;
loadtau=.1*(t>=.5); J=p.Jload_test+p.N^2*p.Jm;
expected=cumtrapz(t,(y-loadtau-p.N^2*p.Bm*v)/J);
mechanicsError=max(abs(v-expected));assert(mechanicsError<1e-3,'Mechanical balance failed');
assert(v(end)>0,'Positive torque must accelerate positive joint coordinate');
fprintf('Independent mechanics balance max speed error = %.9g rad/s; Jjoint = %.9g kg m^2\n',mechanicsError,J);
save('pmsm_independent_test_results.mat','results','allruns','mechanicsError','p');
f=figure('Visible','off','Position',[50 50 1100 800]);
tiledlayout(3,1);nexttile;
r=allruns.reversal;plot(r.torque_log.Time,r.torque_log.Data);ylabel('Joint torque (N m)');grid on;title('Independent torque reversal, imposed zero speed');
nexttile;r=allruns.voltage_recovery;plot(r.diagnostic_log.Time,r.diagnostic_log.Data(:,[2 3]));ylabel('Current (A)');legend('iq actual','iq reference');grid on;title('Voltage-limited operation and recovery (current reference is not hardware protection)');
nexttile;plot(t,v);ylabel('Joint speed (rad/s)');xlabel('Time (s)');grid on;title('Independent load fixture: Jload + N^2 Jmotor, no Gen3 dynamics');
exportgraphics(f,'pmsm_independent_tests.png','Resolution',140);close(f);
close_system(m,0);close_system(h,0);
fprintf('PASS: ten imposed-speed actuator tests and one independent mechanical fixture.\n');
end

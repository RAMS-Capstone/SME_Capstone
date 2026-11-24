"""1-DOF prediction for run_stage3_actuator (independent of MATLAB).

Joint 2 alone, with gravity and joint coupling assumed perfectly compensated by the CTC:
    J_L*qdd = tau_J,  tau_J = Kg*(th - q) + cg*(thd - qd)           (gear spring, joint side)
    Jr*thdd = tau_m - tau_J,  tau_m = 1 kHz first-order lag of tau_cmd (current loop)
    tau_cmd = (J_L + ctc_Jr)*(qdd_ref + Kp*e + Kd*ed),  Kp = 100, Kd = 20 (Gen3 CTC)
Reference: the p2p move of joint 2 (+0.8 rad, quintic, 3 s). J_L = 1.25 kg m^2 (mean M22).
Gives the order of magnitude expected in Gen3: ctc_Jr off -> about 1 mrad, on -> about 0.002 mrad.
"""
import numpy as np
Kg=1.4e4; Jm=170e-7; JWG=0.079e-4; N=100; Jr=N**2*(Jm+JWG); cg=2*0.05*np.sqrt(Kg*Jr)
JL=1.25; Kp=100; Kd=20; wc=2*np.pi*1000
T=3; D=0.8; t0=0.5
def ref(t):
    x=min(max((t-t0)/T,0),1)
    s=10*x**3-15*x**4+6*x**5; sd=(30*x**2-60*x**3+30*x**4)/T; sdd=(60*x-180*x**2+120*x**3)/T**2
    if not (t0<t<t0+T): sd=sdd=0
    return D*s,D*sd,D*sdd
def run(jrc, act=True):
    dt=1e-5; n=int(5/dt); x=np.zeros(5) # q qd th thd tauc_lag
    emax=0; tjmax=0; defmax=0
    for i in range(n):
        t=i*dt
        def f(x,t):
            q,qd,th,thd,tl=x; r,rd,rdd=ref(t)
            tc=(JL+jrc)*(rdd+Kp*(r-q)+Kd*(rd-qd))
            if not act:
                return np.array([qd,tc/JL,0,0,0])
            tj=Kg*(th-q)+cg*(thd-qd)
            return np.array([qd,tj/JL,thd,(tl-tj)/Jr,wc*(tc-tl)])
        k1=f(x,t);k2=f(x+dt/2*k1,t+dt/2);k3=f(x+dt/2*k2,t+dt/2);k4=f(x+dt*k3,t+dt)
        x=x+dt/6*(k1+2*k2+2*k3+k4)
        r=ref(t+dt)[0]; emax=max(emax,abs(r-x[0]))
        if act: tjmax=max(tjmax,abs(Kg*(x[2]-x[0]))); defmax=max(defmax,abs(x[2]-x[0]))
    return emax,tjmax,defmax
print(f"Jr = {Jr:.3f} kg m^2, cg = {cg:.2f} N m s/rad")
for name, jrc, act in [("ideal", 0, False), ("act_nojr", 0, True), ("act_jr", Jr, True)]:
    e, tj, de = run(jrc, act)
    print(f"{name:9s} max error {1e3*e:8.4f} mrad   peak inertial gear torque {tj:5.2f} N m   "
          f"peak deflection {1e3*de:6.3f} mrad")

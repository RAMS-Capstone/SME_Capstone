"""
Stage 2 reference (Python, independent of Simulink): joint-2 actuator on a 1-DOF test bench.

Actuator (same equations as build_stage2_actuator.m / the interface guide):
  iq_cmd = tau_cmd / (s*N*Kt), limited to +-Imax
  FOC (stage 1): PI + decoupling + circular |v| <= Vmax (d priority) + back-calculation anti-windup
  dq PMSM (stage 1), speed input = motor shaft speed wm
  (Jm + J_WG) dwm/dt = Te - (s/N) tau_J,   dthm/dt = wm
  tau_J = Kg (s thm/N - q) + cg (s wm/N - qd)
Bench:
  fixed : q = qd = 0
  free  : J_L qdd = tau_J - G sin(q) - b qd   (pendulum, J_L ~ Gen3 M22)
  ideal : same load driven directly by tau_cmd (for comparison)
"""
import math, json, sys
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

out = sys.argv[1] if len(sys.argv) > 1 else "."

kM_B = 0.0481
P = dict(pp=8, Rs=0.0935/2, L=0.123e-3/2, N=100, Jm=170e-7, JWG=0.079e-4,
         Kg=1.4e4, zeta_g=0.05, Imax=20.9)
P["Kt"] = math.pi/(2*math.sqrt(3))*kM_B
P["psi"] = P["Kt"]/(1.5*P["pp"])
P["Vmax"] = 0.9*24/math.sqrt(3)
wc = 2*math.pi*1000; P["Kp"] = wc*P["L"]; P["Ki"] = wc*P["Rs"]; P["Kaw"] = P["Ki"]/P["Kp"]
P["Jsh"] = P["Jm"] + P["JWG"]
P["cg"] = 2*P["zeta_g"]*math.sqrt(P["Kg"]*P["N"]**2*P["Jsh"])
P["Jr"] = P["N"]**2*P["Jsh"]
B = dict(JL=1.2, G=15.0, b=0.5)          # bench: load inertia, gravity torque amplitude, damping

def foc(xd, xq, id_, iq, iq_cmd, we):
    iq_ref = min(max(iq_cmd, -P["Imax"]), P["Imax"])
    ed, eq = -id_, iq_ref - iq
    vd_raw = P["Kp"]*ed + xd - we*P["L"]*iq
    vq_raw = P["Kp"]*eq + xq + we*(P["L"]*id_ + P["psi"])
    vd = min(max(vd_raw, -P["Vmax"]), P["Vmax"])
    lim = math.sqrt(max(P["Vmax"]**2 - vd**2, 0.0))
    vq = min(max(vq_raw, -lim), lim)
    return vd, vq, iq_ref, (P["Ki"]*ed + P["Kaw"]*(vd - vd_raw), P["Ki"]*eq + P["Kaw"]*(vq - vq_raw))

def quintic(x):
    x = min(max(x, 0.0), 1.0)
    return 10*x**3-15*x**4+6*x**5, 30*x**2-60*x**3+30*x**4, 60*x-180*x**2+120*x**3

def make_ref(t0=0.5, T=3.0, d=0.8):
    def ref(t):
        s, sd, sdd = quintic((t - t0)/T)
        return d*s, d*sd/T, d*sdd/T**2
    return ref

def rhs(t, x, sc, s):
    id_, iq, xd, xq, wm, thm, q, qd, qi, qdi = x
    if sc["fixed"]:
        q = qd = 0.0
    tc_act, tc_ideal = sc["cmd"](t)
    we = P["pp"]*wm
    vd, vq, _, dxi = foc(xd, xq, id_, iq, tc_act/(s*P["N"]*P["Kt"]), we)
    did = (vd - P["Rs"]*id_ + we*P["L"]*iq)/P["L"]
    diq = (vq - P["Rs"]*iq - we*P["L"]*id_ - we*P["psi"])/P["L"]
    Te = 1.5*P["pp"]*P["psi"]*iq
    tauJ = P["Kg"]*(s*thm/P["N"] - q) + P["cg"]*(s*wm/P["N"] - qd)
    dwm = (Te - (s/P["N"])*tauJ)/P["Jsh"]
    if sc["fixed"]:
        dq_, dqd = 0.0, 0.0
    else:
        dq_, dqd = qd, (tauJ - B["G"]*math.sin(q) - B["b"]*qd)/B["JL"]
    dqi, dqdi = qdi, (tc_ideal - B["G"]*math.sin(qi) - B["b"]*qdi)/B["JL"]
    return [did, diq, dxi[0], dxi[1], dwm, wm, dq_, dqd, dqi, dqdi]

def run(sc, s=-1, T=0.6, dt_out=2e-4):
    te = np.arange(0, T + 1e-12, dt_out)
    sol = solve_ivp(rhs, (0, T), np.zeros(10), args=(sc, s), method="LSODA", t_eval=te,
                    max_step=5e-5, rtol=1e-8, atol=1e-10)
    assert sol.success, sol.message
    t = sol.t; X = sol.y.T
    rows = []
    for k in range(len(t)):
        id_, iq, xd, xq, wm, thm, q, qd, qi, qdi = X[k]
        if sc["fixed"]: q = qd = 0.0
        tc_act, _ = sc["cmd"](t[k])
        vd, vq, iq_ref, _ = foc(xd, xq, id_, iq, tc_act/(s*P["N"]*P["Kt"]), P["pp"]*wm)
        tauJ = P["Kg"]*(s*thm/P["N"] - q) + P["cg"]*(s*wm/P["N"] - qd)
        rows.append([iq_ref, iq, id_, vd, vq, math.hypot(vd, vq), wm, 1.5*P["pp"]*P["psi"]*iq,
                     tauJ, s*thm/P["N"] - q, q, qd, qi, qdi])
    return t, np.array(rows)   # columns 0..9 = j2_act_log order, then q, qd, q_ideal, qd_ideal

step = lambda a, t0=0.01: (lambda t: (a, a) if t >= t0 else (0.0, 0.0))
ref = make_ref()
def pend_cmd(aware):
    def cmd(t):
        qr, qdr, qddr = ref(t)
        base = B["JL"]*qddr + B["b"]*qdr + B["G"]*math.sin(qr)
        return (base + (P["Jr"]*qddr if aware else 0.0), base)
    return cmd

tail = lambda t, y, a=0.5: float(np.mean(y[t >= a]))
res = {"params": {k: P[k] for k in ("Kt","N","Kg","cg","Jsh","Jr","Imax","Vmax")}, "bench": B}

t, Y = run(dict(fixed=True, cmd=step(10.0)))
res["T1_static"] = dict(tauJ_tail=tail(t, Y[:,8]), defl_tail=tail(t, Y[:,9]),
                        defl_expected=10/P["Kg"], tauJ_peak=float(Y[:,8].max()), iq_tail=tail(t, Y[:,1]))
t2, Y2 = run(dict(fixed=True, cmd=step(10.0)), s=+1)
res["T2_sign"] = dict(tauJ_maxdiff=float(np.max(np.abs(Y2[:,8]-Y[:,8]))),
                      defl_maxdiff=float(np.max(np.abs(Y2[:,9]-Y[:,9]))),
                      iq_tail_s_minus1=tail(t, Y[:,1]), iq_tail_s_plus1=tail(t2, Y2[:,1]))
t3, Y3 = run(dict(fixed=True, cmd=step(100.0)))
res["T3_limit"] = dict(iq_cmd_max=float(np.max(np.abs(Y3[:,0]))), tauJ_tail=tail(t3, Y3[:,8]),
                       tauJ_expected=P["N"]*P["Kt"]*P["Imax"])
pend = {}
for aware in (False, True):
    tp, Yp = run(dict(fixed=False, cmd=pend_cmd(aware)), T=5.0, dt_out=1e-3)
    e = Yp[:,10] - Yp[:,12]
    key = "T4_pendulum_aware" if aware else "T4_pendulum"
    res[key] = dict(max_q_minus_ideal_mrad=1e3*float(np.max(np.abs(e))),
                    q_end=float(Yp[-1,10]), q_ideal_end=float(Yp[-1,12]),
                    iq_peak=float(np.max(np.abs(Yp[:,1]))), vmag_peak=float(Yp[:,5].max()),
                    defl_peak_mrad=1e3*float(np.max(np.abs(Yp[:,9]))),
                    wm_peak=float(np.max(np.abs(Yp[:,6]))))
    pend[aware] = (tp, Yp)

json.dump(res, open(f"{out}/stage2_reference.json","w"), indent=1)
print(json.dumps(res, indent=1))

fig, ax = plt.subplots(3, 1, figsize=(10, 9))
ax[0].plot(t, Y[:,8], label="tau_J, s=-1"); ax[0].plot(t2, Y2[:,8], "--", label="tau_J, s=+1")
ax[0].plot(t3, Y3[:,8], label="tau_J, 100 N m command (current-limited)")
ax[0].set_ylabel("N m"); ax[0].legend(fontsize=8); ax[0].grid(alpha=.3); ax[0].set_title("T1-T3: joint fixed")
for aware, ls in ((False, "-"), (True, "--")):
    tp, Yp = pend[aware]
    ax[1].plot(tp, 1e3*(Yp[:,10]-Yp[:,12]), ls, label=("with Jr feed-forward" if aware else "same command as ideal"))
ax[1].set_ylabel("q - q_ideal (mrad)"); ax[1].legend(fontsize=8); ax[1].grid(alpha=.3); ax[1].set_title("T4: pendulum")
tp, Yp = pend[False]
ax[2].plot(tp, Yp[:,10], label="q actuator"); ax[2].plot(tp, Yp[:,12], "--", label="q ideal torque")
ax[2].set_ylabel("rad"); ax[2].set_xlabel("time (s)"); ax[2].legend(fontsize=8); ax[2].grid(alpha=.3)
fig.tight_layout(); fig.savefig(f"{out}/stage2_reference.png", dpi=120)

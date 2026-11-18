"""
Gen3 J2 actuator candidate sizing (pre-integration, baseline-trace based).

- Input : joint2_baseline_trace.csv (columns Time_s, TorqueBeforeLimit_Nm,
          TorqueApplied_Nm, ActualVelocity_rad_s, ActualPosition_rad, MechanicalPower_W)
- Output: candidate_sizing_summary.csv, candidate_sizing_sweep.csv,
          candidate_timeseries_<id>.csv (per baseline candidate/nominal efficiency)

All motor-side numbers are ESTIMATES from the baseline joint trace. Adding rotor
inertia changes robot dynamics, so these are not integrated-simulation results.
Joint acceleration is a numerical derivative of the sampled joint velocity.

Usage: python candidate_sizing_j2.py ../data/joint2_baseline_trace.csv <outdir>
"""
import sys, math, json
import numpy as np

csv_path = sys.argv[1] if len(sys.argv) > 1 else "joint2_baseline_trace.csv"
outdir = sys.argv[2] if len(sys.argv) > 2 else "."

d = np.genfromtxt(csv_path, delimiter=",", names=True)
t = d["Time_s"]; tau = d["TorqueApplied_Nm"]; w = d["ActualVelocity_rad_s"]

# --- near-duplicate time handling (merge samples closer than 1e-9 s) ---
keep = np.concatenate(([True], np.diff(t) > 1e-9))
n_dup = int((~keep).sum())
t, tau, w = t[keep], tau[keep], w[keep]
alpha = np.gradient(w, t)             # numerical estimate [rad/s^2]
dt = np.diff(t)

def tw_rms(x):                         # time-weighted RMS (trapezoid)
    return math.sqrt(np.sum(0.5 * (x[1:]**2 + x[:-1]**2) * dt) / (t[-1] - t[0]))

def tw_mean(x):
    return np.sum(0.5 * (x[1:] + x[:-1]) * dt) / (t[-1] - t[0])

# ---------------- manufacturer data (see report for sources) ----------------
motors = {
 "ECi52_180W_48V_579165": dict(R_pp=0.284, L_pp=0.45e-3, kM_B=0.0919, IN_B=4.59, MN_B=0.423,
                               J=170e-7, pp=8, nmax_rpm=6000, Vnom=48, n0_rpm=4950),
 "ECi52_200W_48V_634043": dict(R_pp=0.236, L_pp=0.424e-3, kM_B=0.115, IN_B=5.48, MN_B=0.627,
                               J=264e-7, pp=8, nmax_rpm=5000, Vnom=48, n0_rpm=3970),
 "DT50M_210W_48V_758031": dict(R_pp=0.748, L_pp=0.571e-3, kM_B=0.105, IN_B=5.16, MN_B=0.600,
                               J=98.8e-7, pp=10, nmax_rpm=float("nan"), Vnom=48, n0_rpm=4330),
 # 24 V windings (catalog columns) for a 24 V bus sensitivity check
 "ECi52_180W_24V_574741": dict(R_pp=0.0935, L_pp=0.123e-3, kM_B=0.0481, IN_B=9.03, MN_B=0.438,
                               J=170e-7, pp=8, nmax_rpm=6000, Vnom=24, n0_rpm=4720),
 "ECi52_200W_24V_606793": dict(R_pp=0.117, L_pp=0.149e-3, kM_B=0.068, IN_B=9.42, MN_B=0.646,
                               J=264e-7, pp=8, nmax_rpm=5000, Vnom=24, n0_rpm=3340),
}
# Harmonic Drive CSG-17 component set (official product pages, -LW variants)
gears = {
  50: dict(rated=21, avg=34, rep_peak=44, mom=91,  n_avg_rpm=3500),
  80: dict(rated=29, avg=35, rep_peak=56, mom=113, n_avg_rpm=3500),
 100: dict(rated=31, avg=51, rep_peak=70, mom=125, n_avg_rpm=3500),
}
J_WG = 0.079e-4      # CSG-17 input inertia, brochure text extract -> to be re-confirmed
N_MAX_IN_RPM = 7300  # CSG-17 max input speed, brochure text extract -> to be re-confirmed

# block -> sinusoidal (FOC) conversion, maxon academy:
#   kM_S = pi/(2*sqrt3) * kM_B  (S = phase-current amplitude),  IN_S = 2/sqrt3 * IN_B
K_KM = math.pi / (2 * math.sqrt(3)); K_IN = 2 / math.sqrt(3)
ALPHA_CU = 0.0039   # copper temperature coefficient [1/K] (assumption)
P_STATIC = 1e-3     # |P| below this [W] -> treat as static holding

def evaluate(mname, N, eta_f, eta_b, Vdc_list=(48.0, 24.0), hot=False):
    m = motors[mname]
    Rs = m["R_pp"] / 2 * ((1 + ALPHA_CU * 130) if hot else 1)   # wye assumption, 25C or 155C
    Ls = m["L_pp"] / 2                                          # Ld = Lq assumption
    Kt = K_KM * m["kM_B"]                                       # N*m per A (amplitude)
    psi = Kt / (1.5 * m["pp"])
    Jm = m["J"] + J_WG
    P = tau * w
    drive = P > P_STATIC; regen = P < -P_STATIC; static = ~(drive | regen)
    T_load = np.where(drive, tau / (N * eta_f),
             np.where(regen, tau * eta_b / N, tau / (N * eta_f)))  # static: conservative
    T_inert = Jm * N * alpha                                    # motor-side rotor+WG inertia
    Tm = T_load + T_inert
    iq = Tm / Kt
    wm = N * w; we = m["pp"] * wm
    diq = np.gradient(iq, t)
    vq = Rs * iq + Ls * diq + we * psi
    vd = -we * Ls * iq
    vmag = np.hypot(vd, vq)
    vmag_ss = np.hypot(-we * Ls * iq, Rs * iq + we * psi)       # quasi-steady (no L di/dt)
    # gear average torque (HD method, output torque, input speed weighted cube mean)
    n_in = np.abs(wm) * 60 / (2 * math.pi)
    wgt = 0.5 * (n_in[1:] + n_in[:-1]) * dt
    T3 = 0.5 * (np.abs(tau[1:])**3 + np.abs(tau[:-1])**3)
    T_av_gear = (np.sum(wgt * T3) / np.sum(wgt)) ** (1 / 3)
    n_av = tw_mean(n_in)
    g = gears[N]
    res = dict(motor=mname, N=N, eta_f=eta_f, eta_b=eta_b, hot=hot,
        Kt_S=Kt, psi=psi, Rs=Rs, Ls=Ls, Jm_total=Jm, J_reflected_joint=N**2 * Jm,
        n_motor_max_rpm=float(np.max(n_in)), n_motor_max_ratio_to_nmax=float(np.max(n_in) / m["nmax_rpm"]),
        Tm_peak=float(np.max(np.abs(Tm))), Tm_rms=tw_rms(Tm),
        Tinert_peak=float(np.max(np.abs(T_inert))), Tinert_joint_equiv_peak=float(np.max(np.abs(T_inert)) * N),
        iq_peak=float(np.max(np.abs(iq))), iq_rms=tw_rms(iq),
        IN_S=K_IN * m["IN_B"], MN_S=1.05 * m["MN_B"],
        Tm_rms_over_MN_S=tw_rms(Tm) / (1.05 * m["MN_B"]),
        Tm_peak_over_MN_S=float(np.max(np.abs(Tm))) / (1.05 * m["MN_B"]),
        iq_hold_end=float(iq[-1]),
        v_peak=float(np.max(vmag)), v_peak_ss=float(np.max(vmag_ss)),
        v_peak_t=float(t[np.argmax(vmag)]),
        gear_out_peak=float(np.max(np.abs(tau))), gear_rep_peak=g["rep_peak"],
        gear_T_av=T_av_gear, gear_avg_limit=g["avg"], gear_rated=g["rated"],
        gear_n_avg_rpm=n_av, gear_n_avg_limit=g["n_avg_rpm"],
        gear_n_in_max_over_limit=float(np.max(n_in)) / N_MAX_IN_RPM,
        P_elec_min=float(np.min(1.5 * (vq * iq + vd * 0))),   # = 1.5*vq*iq (id=0)
        E_elec_neg=float(-np.sum(np.minimum(0, 0.5 * ((1.5 * vq * iq)[1:] + (1.5 * vq * iq)[:-1])) * dt)),
        E_elec_pos=float(np.sum(np.maximum(0, 0.5 * ((1.5 * vq * iq)[1:] + (1.5 * vq * iq)[:-1])) * dt)),
    )
    for Vdc in Vdc_list:
        res[f"Vlim_{int(Vdc)}V"] = Vdc / math.sqrt(3)
        res[f"v_margin_{int(Vdc)}V"] = Vdc / math.sqrt(3) - res["v_peak"]
    ts = dict(t=t, tau=tau, w=w, alpha=alpha, Tm=Tm, Tinert=T_inert, iq=iq, wm_rpm=n_in * np.sign(wm), vd=vd, vq=vq, vmag=vmag)
    return res, ts

rows = []; ts_store = {}
for mname in motors:
    for N in gears:
        for eta in (0.6, 0.7, 0.8):
            for eta_b in (eta, 1.0):
                for hot in (False, True):
                    r, ts = evaluate(mname, N, eta, eta_b, hot=hot)
                    rows.append(r)
                    if eta == 0.7 and eta_b == 0.7 and not hot:
                        ts_store[(mname, N)] = ts

keys = list(rows[0].keys())
with open(f"{outdir}/candidate_sizing_sweep.csv", "w") as f:
    f.write(",".join(keys) + "\n")
    for r in rows:
        f.write(",".join(str(r[k]) for k in keys) + "\n")

for (mname, N), ts in ts_store.items():
    arr = np.column_stack([ts[k] for k in ts])
    np.savetxt(f"{outdir}/ts_{mname}_N{N}.csv", arr, delimiter=",", header=",".join(ts.keys()), comments="")

meta = dict(n_samples=int(len(t)), n_merged_duplicates=n_dup, dt_max=float(dt.max()),
            alpha_peak=float(np.max(np.abs(alpha))), alpha_peak_t=float(t[np.argmax(np.abs(alpha))]),
            tau_peak=float(np.max(np.abs(tau))), w_peak=float(np.max(np.abs(w))))
json.dump(meta, open(f"{outdir}/trace_meta.json", "w"), indent=1)
print(json.dumps(meta, indent=1))

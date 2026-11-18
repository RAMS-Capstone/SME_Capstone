"""
Stage 1 independent check (Python, not Simulink).

Compares two separately written PMSM plant models driven by the SAME FOC current controller:
  (A) dq model  : the equations the Simulink 'PMSM_dq_manual' subsystem implements
  (B) abc model : three phase equations + back-EMF + energy-based torque, controller sees
                  currents only through Clarke/Park (as a real drive does)
If (A) and (B) agree, the dq derivation and the Park/inverse-Park conventions are consistent.
Parameters = j2_params.m (maxon EC-i 52 180W 24V #574741, see 관절2_기준사양_결정.md).
"""
import math, json, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

out = sys.argv[1] if len(sys.argv) > 1 else "."

# ---------------- parameters (keep identical to j2_params.m) ----------------
kM_B = 0.0481
P = dict(
    pp=8,
    Rs=0.0935 / 2,                 # wye assumption, 25 C
    L=0.123e-3 / 2,                # Ld = Lq (non-salient assumption)
    Kt=math.pi / (2 * math.sqrt(3)) * kM_B,
    Jm=170e-7, JWG=0.079e-4, N=100,
    Vdc=24.0, Imax=20.9,
)
P["psi"] = P["Kt"] / (1.5 * P["pp"])
P["Vmax"] = 0.9 * P["Vdc"] / math.sqrt(3)
wc = 2 * math.pi * 1000
P.update(Kp=wc * P["L"], Ki=wc * P["Rs"])
P["Kaw"] = P["Ki"] / P["Kp"]
A120 = 2 * math.pi / 3

def controller(x_i, id_, iq, iq_cmd, we):
    """FOC current PI + decoupling + circular voltage limit (d priority) + back-calc anti-windup."""
    xd, xq = x_i
    iq_ref = min(max(iq_cmd, -P["Imax"]), P["Imax"])
    ed, eq = 0.0 - id_, iq_ref - iq
    vd_raw = P["Kp"] * ed + xd - we * P["L"] * iq
    vq_raw = P["Kp"] * eq + xq + we * (P["L"] * id_ + P["psi"])
    vd = min(max(vd_raw, -P["Vmax"]), P["Vmax"])
    vq_lim = math.sqrt(max(P["Vmax"] ** 2 - vd ** 2, 0.0))
    vq = min(max(vq_raw, -vq_lim), vq_lim)
    dxd = P["Ki"] * ed + P["Kaw"] * (vd - vd_raw)
    dxq = P["Ki"] * eq + P["Kaw"] * (vq - vq_raw)
    return vd, vq, vq_raw, iq_ref, (dxd, dxq)

def park(ia, ib, ic, th):
    d = 2 / 3 * (ia * math.cos(th) + ib * math.cos(th - A120) + ic * math.cos(th + A120))
    q = -2 / 3 * (ia * math.sin(th) + ib * math.sin(th - A120) + ic * math.sin(th + A120))
    return d, q

def inv_park(vd, vq, th):
    return [vd * math.cos(th - ph) - vq * math.sin(th - ph) for ph in (0, A120, -A120)]

# ---- state vectors -------------------------------------------------------
# dq : [id, iq, thm, wm, xd, xq]      abc : [ia, ib, thm, wm, xd, xq]
def f_dq(t, x, sc):
    id_, iq, thm, wm, xd, xq = x
    if sc["speed"] is not None:
        wm = sc["speed"](t)
    we = P["pp"] * wm
    vd, vq, *_ , dxi = controller((xd, xq), id_, iq, sc["iq_cmd"](t), we)
    did = (vd - P["Rs"] * id_ + we * P["L"] * iq) / P["L"]
    diq = (vq - P["Rs"] * iq - we * P["L"] * id_ - we * P["psi"]) / P["L"]
    Te = 1.5 * P["pp"] * P["psi"] * iq
    dwm = 0.0 if sc["speed"] is not None else (Te - sc["TL"](t)) / sc["J"]
    return np.array([did, diq, wm, dwm, *dxi])

def f_abc(t, x, sc):
    ia, ib, thm, wm, xd, xq = x
    ic = -ia - ib
    if sc["speed"] is not None:
        wm = sc["speed"](t)
    th = P["pp"] * thm; we = P["pp"] * wm
    id_, iq = park(ia, ib, ic, th)
    vd, vq, *_ , dxi = controller((xd, xq), id_, iq, sc["iq_cmd"](t), we)
    va, vb, vc = inv_park(vd, vq, th)
    e = [-we * P["psi"] * math.sin(th - ph) for ph in (0, A120, -A120)]   # d(psi_pm,k)/dt
    vn = (va + vb + vc) / 3 - sum(e) / 3          # isolated neutral
    dia = (va - vn - P["Rs"] * ia - e[0]) / P["L"]
    dib = (vb - vn - P["Rs"] * ib - e[1]) / P["L"]
    Te = -P["pp"] * P["psi"] * (ia * math.sin(th) + ib * math.sin(th - A120) + ic * math.sin(th + A120))
    dwm = 0.0 if sc["speed"] is not None else (Te - sc["TL"](t)) / sc["J"]
    return np.array([dia, dib, wm, dwm, *dxi])

def rk4(f, x0, T, dt, sc, every=10):
    n = int(round(T / dt)); x = np.array(x0, float); rows = []
    for k in range(n + 1):
        t = k * dt
        if k % every == 0:
            rows.append((t, *x))
        k1 = f(t, x, sc); k2 = f(t + dt / 2, x + dt / 2 * k1, sc)
        k3 = f(t + dt / 2, x + dt / 2 * k2, sc); k4 = f(t + dt, x + dt * k3, sc)
        x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.array(rows)

def post(rows, sc, kind):
    """Recompute id, iq, Te, v from logged states."""
    res = dict(t=[], id=[], iq=[], Te=[], wm=[], vd=[], vq=[], vq_raw=[], vmag=[], xq=[], iq_ref=[])
    for t, a, b, thm, wm, xd, xq in rows:
        if sc["speed"] is not None: wm = sc["speed"](t)
        th = P["pp"] * thm; we = P["pp"] * wm
        if kind == "dq": id_, iq = a, b
        else: id_, iq = park(a, b, -a - b, th)
        vd, vq, vq_raw, iq_ref, _ = controller((xd, xq), id_, iq, sc["iq_cmd"](t), we)
        for k, v in zip(res, (t, id_, iq, 1.5 * P["pp"] * P["psi"] * iq, wm, vd, vq, vq_raw, math.hypot(vd, vq), xq, iq_ref)):
            res[k].append(v)
    return {k: np.array(v) for k, v in res.items()}

step = lambda t0, a: (lambda t: a if t >= t0 else 0.0)
J_test = P["Jm"] + P["JWG"] + 0.5 / P["N"] ** 2   # motor+WG + 0.5 kg m^2 joint-side test load (assumption)
scen = {
  "T1_locked_iq_step":     dict(iq_cmd=step(1e-3, 10.0), speed=lambda t: 0.0, T=6e-3),
  "T2_rated_speed_iq":     dict(iq_cmd=step(1e-3, 10.0), speed=lambda t: 270.5, T=6e-3),   # 2583 rpm = baseline peak
  "T3_voltage_limit":      dict(iq_cmd=step(1e-3, 10.0),
                                speed=lambda t: 470.0 if 4e-3 <= t < 12e-3 else 270.5, T=20e-3),
  "T4_free_rotor":         dict(iq_cmd=lambda t: 10.0 if 1e-3 <= t < 0.15 else (-10.0 if t >= 0.15 else 0.0),
                                speed=None, J=J_test, TL=lambda t: 0.0, T=0.3),
  "T5_command_limit":      dict(iq_cmd=step(1e-3, 30.0), speed=lambda t: 0.0, T=4e-3),
}

summary = {"params": {k: P[k] for k in ("pp", "Rs", "L", "Kt", "psi", "Vmax", "Kp", "Ki", "Kaw", "Imax")}}
fig, axs = plt.subplots(len(scen), 2, figsize=(12, 2.6 * len(scen)))
for row, (name, sc) in enumerate(scen.items()):
    dt = 1e-6 if sc["T"] <= 0.03 else 4e-6
    every = 5 if sc["T"] <= 0.03 else 50
    rd = post(rk4(f_dq, [0, 0, 0, 0, 0, 0], sc["T"], dt, sc, every), sc, "dq")
    ra = post(rk4(f_abc, [0, 0, 0, 0, 0, 0], sc["T"], dt, sc, every), sc, "abc")
    s = dict(
        max_abs_diff_iq_A=float(np.max(np.abs(rd["iq"] - ra["iq"]))),
        max_abs_diff_id_A=float(np.max(np.abs(rd["id"] - ra["id"]))),
        max_abs_diff_Te_Nm=float(np.max(np.abs(rd["Te"] - ra["Te"]))),
        max_abs_diff_wm=float(np.max(np.abs(rd["wm"] - ra["wm"]))),
        iq_final=float(rd["iq"][-1]), id_max_abs=float(np.max(np.abs(rd["id"]))),
        vmag_max=float(np.max(rd["vmag"])), vmag_limit=P["Vmax"],
        iq_ref_max=float(np.max(np.abs(rd["iq_ref"]))),
        xq_max_abs=float(np.max(np.abs(rd["xq"]))),
    )
    t, iq = rd["t"], rd["iq"]
    if name in ("T1_locked_iq_step", "T2_rated_speed_iq"):
        a = iq >= 1.0; b = iq >= 9.0
        s["rise_10_90_ms"] = float((t[np.argmax(b)] - t[np.argmax(a)]) * 1e3)
        s["overshoot_pct"] = float((iq.max() - 10) / 10 * 100)
    if name == "T3_voltage_limit":
        sat = t[(rd["vq_raw"] - rd["vq"]) > 1e-3]
        s["saturated_from_to_ms"] = [float(sat.min() * 1e3), float(sat.max() * 1e3)] if len(sat) else None
        s["iq_during_overspeed_min"] = float(iq[(t > 6e-3) & (t < 12e-3)].min())
        rec = t[(t > 12e-3) & (np.abs(iq - 10) < 0.2)]
        s["recovered_within_ms_after_12ms"] = float((rec.min() - 12e-3) * 1e3) if len(rec) else None
        s["iq_overshoot_after_recovery_pct"] = float((iq[t > 12e-3].max() - 10) / 10 * 100)
    if name == "T4_free_rotor":
        s["wm_at_20ms"] = float(np.interp(0.02, t, rd["wm"]))
        s["wm_at_20ms_expected_Te_over_J"] = float(1.5 * P["pp"] * P["psi"] * 10 * (0.02 - 1e-3) / J_test)
        s["wm_peak"] = float(rd["wm"].max())
        s["wm_voltage_limited_theory"] = float(P["Vmax"] / (P["pp"] * P["psi"]))
        s["J_test"] = J_test
    summary[name] = s
    ms = 1e3
    axs[row, 0].plot(t * ms, ra["iq"], lw=3, alpha=.35, label="iq abc model")
    axs[row, 0].plot(t * ms, rd["iq"], "--", label="iq dq model")
    axs[row, 0].plot(t * ms, rd["id"], label="id dq")
    axs[row, 0].plot(t * ms, rd["iq_ref"], ":", c="k", label="iq_ref (limited)")
    axs[row, 0].set_ylabel("A"); axs[row, 0].set_title(name, fontsize=9); axs[row, 0].grid(alpha=.3)
    axs[row, 1].plot(t * ms, rd["vmag"], label="|v| applied")
    axs[row, 1].plot(t * ms, np.hypot(rd["vd"], rd["vq_raw"]), ":", label="|v| requested")
    axs[row, 1].axhline(P["Vmax"], c="r", lw=.8, label="limit 12.47 V")
    if name == "T4_free_rotor":
        ax2 = axs[row, 1].twinx(); ax2.plot(t * ms, rd["wm"], c="g", lw=.8); ax2.set_ylabel("wm rad/s", color="g")
    axs[row, 1].set_ylabel("V"); axs[row, 1].grid(alpha=.3)
    if row == 0:
        axs[row, 0].legend(fontsize=7); axs[row, 1].legend(fontsize=7)
for a in axs[-1]: a.set_xlabel("time (ms)")
fig.tight_layout(); fig.savefig(f"{out}/stage1_verification.png", dpi=130)
json.dump(summary, open(f"{out}/stage1_verification.json", "w"), indent=1, ensure_ascii=False)
print(json.dumps(summary, indent=1, ensure_ascii=False))

"""
Reads the blk(...) / wiring lists in build_stage1_pmsm.m, executes the block diagram they describe
(Gain, Sum, Product, Integrator, Saturation, Saturation Dynamic, Sqrt, Constant, Mux, Switch),
and re-runs tests T1-T5. If results equal stage1_verification.json, the .m wiring is right.
"""
import re, math, json, sys
import numpy as np

src = open(sys.argv[1] if len(sys.argv) > 1 else "build_stage1_pmsm.m", encoding="utf-8").read()
kM_B = 0.0481
p = dict(pp=8, Rs=0.0935/2, Ld=0.123e-3/2); p["Lq"] = p["Ld"]
p["Kt"] = math.pi/(2*math.sqrt(3))*kM_B; p["psi"] = p["Kt"]/(1.5*p["pp"])
p["Vmax"] = 0.9*24/math.sqrt(3); p["Imax"] = 20.9
wc = 2*math.pi*1000; p["Kp"] = wc*p["Lq"]; p["Ki"] = wc*p["Rs"]; p["Kaw"] = p["Ki"]/p["Kp"]
p["J_test"] = 170e-7 + 0.079e-4 + 0.5/100**2

def ev(expr):
    e = expr.replace("^", "**").replace("inf", "float('inf')")
    e = re.sub(r"p\.(\w+)", r"p['\1']", e)
    return eval(e, {"p": p, "float": float})

def section(name_start, name_end):
    i = src.index(name_start); j = src.index(name_end, i + 1) if name_end else len(src)
    return src[i:j]

def parse(sec, sysvar):
    blocks = {}
    body = sec
    # expand the d/q for-loop
    loop = re.search(r"for ax = \{'d','q'\}(.*?)\nend", body, re.S)
    if loop:
        inner = loop.group(1)
        exp = ""
        for a in "dq":
            t = inner.replace("['e' a]", f"'e{a}'").replace("['Kp_' a]", f"'Kp_{a}'").replace("['Ki_' a]", f"'Ki_{a}'")
            t = t.replace("['x' a '_drive']", f"'x{a}_drive'").replace("['x' a]", f"'x{a}'").replace("['PI_' a]", f"'PI_{a}'")
            t = t.replace("['v' a '_excess']", f"'v{a}_excess'").replace("['Kaw_' a]", f"'Kaw_{a}'")
            exp += t
        body = body.replace(loop.group(0), exp)
    for mm in re.finditer(r"blk\(" + sysvar + r",'([^']+)','([^']+)'((?:,'[^']*')*)\)", body):
        typ, name, rest = mm.groups()
        kv = re.findall(r"'([^']*)'", rest)
        blocks[name] = dict(type=typ, **dict(zip(kv[::2], kv[1::2])))
    edges = re.findall(r"'([\w]+)/(\d+)','([\w]+)/(\d+)'", body)
    edges += re.findall(r"wire\(" + sysvar + r",'([\w]+)/(\d+)','([\w]+)/(\d+)'\)", body)
    return blocks, list(dict.fromkeys((a, int(b), c, int(d)) for a, b, c, d in edges))

B_pm, E_pm = parse(section("%% ---------------- PMSM_dq_manual", "%% ---------------- FOC_current_ctrl"), "s")
B_c,  E_c  = parse(section("%% ---------------- FOC_current_ctrl", "%% ---------------- root harness"), "c")
B_r,  E_r  = parse(section("%% ---------------- root harness", "% ---------------- helpers"), "m")
B_r["PMSM_dq_manual"] = dict(type="Sub"); B_r["FOC_current_ctrl"] = dict(type="Sub")

class Diagram:
    def __init__(s, B, E, subs=None):
        s.B, s.subs = B, subs or {}
        s.src = {}                                  # (dst, port) -> (src, port)
        for a, ap, b, bp in E:
            assert (b, bp) not in s.src, f"input {b}/{bp} driven twice"
            s.src[(b, bp)] = (a, ap)
        s.ints = [n for n, b in B.items() if b["type"] == "Integrator"]
        s.inports = sorted([n for n, b in B.items() if b["type"] == "In1"], key=lambda n: int(B[n]["Port"]))
        s.outports = sorted([n for n, b in B.items() if b["type"] == "Out1"], key=lambda n: int(B[n]["Port"]))
        for (b, bp), (a, ap) in s.src.items():
            assert a in B and b in B, f"unknown block in edge {a}->{b}"
    def nstates(s):
        return len(s.ints) + sum(d.nstates() for d in s.subs.values())
    def ev_obj(s, x, u):
        """Lazy evaluator: inputs u are callables, so only what an output needs is evaluated."""
        D = s; st = {}; k = 0
        for n in s.ints: st[n] = x[k]; k += 1
        subx = {}
        for n, d in s.subs.items():
            subx[n] = x[k:k + d.nstates()]; k += d.nstates()
        cache = {}; subev = {}
        def inp(b, port):
            a, ap = D.src[(b, port)]
            return val(a, ap)
        def sub(n):
            if n not in subev:
                d = D.subs[n]
                subev[n] = d.ev_obj(subx[n], {nm: (lambda i=i: inp(n, i + 1)) for i, nm in enumerate(d.inports)})
            return subev[n]
        def val(n, port=1):
            if (n, port) in cache: return cache[(n, port)]
            b = D.B[n]; t = b["type"]
            if t == "In1": v = u[n]()
            elif t == "Integrator": v = st[n]
            elif t == "Constant": v = ev(b["Value"]) if b["Value"] != "free_rotor" else u["__free"]()
            elif t == "Gain": v = ev(b["Gain"]) * inp(n, 1)
            elif t == "Sum":
                signs = b.get("Inputs", "++").replace("|", "")
                v = sum((1 if c == "+" else -1) * inp(n, i + 1) for i, c in enumerate(signs))
            elif t == "Product": v = inp(n, 1) * inp(n, 2)
            elif t == "Sqrt": v = math.sqrt(inp(n, 1))
            elif t == "Saturation": v = min(max(inp(n, 1), ev(b["LowerLimit"])), ev(b["UpperLimit"]))
            elif t == "Saturation Dynamic": v = min(max(inp(n, 2), inp(n, 3)), inp(n, 1))
            elif t == "Switch": v = inp(n, 1) if inp(n, 2) != 0 else inp(n, 3)
            elif t == "Mux":
                v = []
                for i in range(int(b["Inputs"])):
                    w = inp(n, i + 1); v += (w if isinstance(w, list) else [w])
            elif t == "Sub": v = sub(n)["out"](port)
            else: raise ValueError(t)
            cache[(n, port)] = v
            return v
        def out(port): return inp(D.outports[port - 1], 1)
        def dx():
            r = [inp(n, 1) for n in D.ints]
            for n in D.subs: r += list(sub(n)["dx"]())
            return np.array(r, float)
        return {"out": out, "dx": dx}
    def run(s, x, u):
        e = s.ev_obj(x, {k: (lambda v=v: v) for k, v in u.items()})
        return [e["out"](i + 1) for i in range(len(s.outports))], e["dx"]()

pm = Diagram(B_pm, E_pm); foc = Diagram(B_c, E_c)
root = Diagram(B_r, E_r, subs={"PMSM_dq_manual": pm, "FOC_current_ctrl": foc})
B_r["stage1_log"] = dict(type="Out1", Port="1"); root.outports = ["stage1_log"]

def simulate(iq_cmd, wm_imp, free, T, dt, every):
    x = np.zeros(root.nstates()); rows = []
    n = int(round(T / dt))
    f = lambda t, x: root.run(x, {"iq_cmd": iq_cmd(t), "wm_imposed": wm_imp(t), "__free": free})
    for k in range(n + 1):
        t = k * dt
        if k % every == 0: rows.append([t] + f(t, x)[0][0])
        k1 = f(t, x)[1]; k2 = f(t + dt/2, x + dt/2*k1)[1]; k3 = f(t + dt/2, x + dt/2*k2)[1]; k4 = f(t + dt, x + dt*k3)[1]
        x = x + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    return np.array(rows)

step = lambda t0, a: (lambda t: a if t >= t0 else 0.0)
ref = json.load(open("stage1_verification.json"))
def cross(t, y, lv):
    k = int(np.argmax(y >= lv)); return t[k-1] + (lv - y[k-1]) * (t[k]-t[k-1]) / (y[k]-y[k-1])
res = {}
# columns: t | 1 id 2 iq 3 Te 4 wm 5 vd 6 vq 7 theta 8 iq_ref 9 vd_raw 10 vq_raw 11 vmag 12 xd 13 xq
r = simulate(step(1e-3, 10), lambda t: 0.0, 0, 6e-3, 1e-6, 5)
res["T1_rise_ms"] = ((cross(r[:,0], r[:,2], 9) - cross(r[:,0], r[:,2], 1)) * 1e3, ref["T1_locked_iq_step"]["rise_10_90_ms"])
r = simulate(step(1e-3, 10), lambda t: 270.5, 0, 6e-3, 1e-6, 5)
res["T2_vmag_max"] = (r[:,11].max(), ref["T2_rated_speed_iq"]["vmag_max"])
r = simulate(step(1e-3, 10), lambda t: 470.0 if 4e-3 <= t < 12e-3 else 270.5, 0, 20e-3, 1e-6, 5)
res["T3_iq_min"] = (r[:,2].min(), ref["T3_voltage_limit"]["iq_during_overspeed_min"])
res["T3_vmag_max"] = (r[:,11].max(), ref["T3_voltage_limit"]["vmag_max"])
r = simulate(lambda t: 10.0 if 1e-3 <= t < 0.15 else (-10.0 if t >= 0.15 else 0.0), lambda t: 0.0, 1, 0.3, 4e-6, 50)
res["T4_wm_20ms"] = (float(np.interp(0.02, r[:,0], r[:,4])), ref["T4_free_rotor"]["wm_at_20ms"])
res["T4_wm_peak"] = (r[:,4].max(), ref["T4_free_rotor"]["wm_peak"])
r = simulate(step(1e-3, 30), lambda t: 0.0, 0, 4e-3, 1e-6, 5)
res["T5_iq_final"] = (r[-1,2], ref["T5_command_limit"]["iq_final"])
ok = True
for k, (a, b) in res.items():
    good = abs(a - b) <= (1e-3 if k == "T1_rise_ms" else 1e-6 * max(1, abs(b)) + 1e-9)  # ref rise used sample index (5 us grid)
    ok &= good
    print(f"{k:14s} wiring-run {a:12.6f}   python-ref {b:12.6f}   {'OK' if good else 'DIFF'}")
print("blocks:", len(B_pm), len(B_c), len(B_r), " edges:", len(E_pm), len(E_c), len(E_r))
print("WIRING MATCHES REFERENCE" if ok else "WIRING DIFFERS")

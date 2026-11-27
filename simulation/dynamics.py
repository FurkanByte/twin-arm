"""TwinArm-5 physical simulations (MuJoCo): holding, demo motion, grasping, thermal/electrical.

    cad run --with matplotlib simulation/dynamics.py

Output: simulation/results/dynamics.json + *.png. Each scenario is reported against a PASS/FAIL criterion.
The servo model is in simulation/mujoco_model.py; electrical/thermal parameters are below (estimates, see README).
"""
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cad"))

import build123d  # noqa: F401,E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

from mujoco_model import ARM_JOINTS, KGCM, NO_LOAD_SPEED, SERVO_OF, twinarm_mujoco  # noqa: E402
from lib.dims import STALL_KGCM, SUPPLY_V  # noqa: E402

OUT = ROOT / "simulation" / "results"
OUT.mkdir(exist_ok=True)

# --- Electrical / thermal parameters (estimates) -------------------------------------------
STALL_CURRENT = {"DS3218MG": 2.5, "DS3230MG": 3.0, "RDS3235": 3.2, "MG90S": 0.7}    # A @6 V
IDLE_CURRENT = 0.08                                                   # A, signal/electronics
R_TH = 14.0          # K/W  servo body -> ambient (60 g plastic case, enclosed pocket)
C_TH = 45.0          # J/K  servo heat capacity
T_AMB = 25.0
T_WARN, T_LIMIT = 50.0, 65.0
RAIL = {"rail_1 (J2,J4,J6)": ["shoulder", "wrist_pitch", "gripper_pinion"],
        "rail_2 (J1,J3,J5)": ["base_yaw", "elbow", "wrist_roll"]}
XL4016_CONT = 5.0    # A, safe continuous current of the heatsinked XL4016 (8 A peak)

from arm import DEMO  # noqa: E402


def stall_nm(j):
    return STALL_KGCM[SERVO_OF[j]] * KGCM


def build(payload_g=0.0, obj=None):
    """Arm + floor (+ payload carried at the TCP) (+ free object on the table)."""
    s = twinarm_mujoco()
    s.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[1, 1, 0.01],
                         rgba=[0.8, 0.8, 0.8, 1], friction=[0.8, 0.01, 0.0001])
    probe = s.compile()
    sid = probe.site("tcp").id
    tcp_body = probe.body(probe.site_bodyid[sid]).name
    tcp_pos = probe.site_pos[sid].copy()
    if payload_g > 0:
        b = s.body(tcp_body).add_body(name="payload", pos=tcp_pos)
        b.mass = payload_g / 1000
        b.inertia = [1e-6, 1e-6, 1e-6]
        b.explicitinertial = True
    if obj is not None:
        size, mass_g, pos = obj
        b = s.worldbody.add_body(name="object", pos=pos)
        b.add_freejoint()
        g = b.add_geom(name="object", type=mujoco.mjtGeom.mjGEOM_BOX, size=[x / 2 for x in size],
                       mass=mass_g / 1000, rgba=[0.2, 0.6, 0.9, 1], friction=[0.8, 0.01, 0.0001],
                       condim=4, solref=[0.004, 1])
    m = s.compile()
    return m, mujoco.MjData(m)


def ids(m):
    q = {j: m.jnt_qposadr[m.joint(j).id] for j in ARM_JOINTS}
    v = {j: m.jnt_dofadr[m.joint(j).id] for j in ARM_JOINTS}
    a = {j: m.actuator(f"servo_{j}").id for j in ARM_JOINTS}
    return q, v, a


def ease(a, b, s):
    s = min(max(s, 0.0), 1.0)
    return a + (3 * s * s - 2 * s ** 3) * (b - a)


def key_targets(m, name):
    k = m.key(name)
    q, _, _ = ids(m)
    return {j: float(k.qpos[q[j]]) for j in ARM_JOINTS}


def robot_bodies(m):
    return {i for i in range(1, m.nbody) if m.body(i).name.endswith("_link")}


def bad_contacts(m, d):
    """Arm self-contacts (non-adjacent links) or contacts with the table (excluding the base)."""
    rb = robot_bodies(m)
    out = []
    for c in d.contact[:d.ncon]:
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        n1, n2 = m.body(b1).name, m.body(b2).name
        if "object" in (n1, n2):
            continue
        if b1 in rb and b2 in rb:
            out.append((n1, n2, float(c.dist)))
        elif (b1 == 0 or b2 == 0):
            other = n2 if b1 == 0 else n1
            if other != "base_link" and other in {m.body(i).name for i in rb}:
                out.append(("floor", other, float(c.dist)))
    return out


class Recorder:
    def __init__(self, m):
        self.m = m
        self.q, self.v, self.a = ids(m)
        self.t, self.tau, self.err, self.qd, self.cmd = [], [], [], [], []

    def __call__(self, d, targets):
        self.t.append(d.time)
        self.tau.append([d.actuator_force[self.a[j]] for j in ARM_JOINTS])
        self.err.append([targets[j] - d.qpos[self.q[j]] for j in ARM_JOINTS])
        self.qd.append([d.qvel[self.v[j]] for j in ARM_JOINTS])

    def arrays(self):
        return (np.array(self.t), np.array(self.tau), np.array(self.err), np.array(self.qd))


def run_profile(m, d, target_fn, t_end, rec=None, dt_rec=0.005, contacts=None):
    q, _, a = ids(m)
    next_rec = 0.0
    while d.time < t_end:
        tg = target_fn(d.time)
        for j in ARM_JOINTS:
            d.ctrl[a[j]] = tg[j]
        mujoco.mj_step(m, d)
        if d.time >= next_rec:
            if rec is not None:
                rec(d, tg)
            if contacts is not None:
                contacts.extend(bad_contacts(m, d))
            next_rec += dt_rec


def set_pose(m, d, targets):
    q, _, a = ids(m)
    for j in ARM_JOINTS:
        d.qpos[q[j]] = targets[j]
        d.ctrl[a[j]] = targets[j]
    mujoco.mj_forward(m, d)


def summarize(rec, window=None):
    t, tau, err, qd = rec.arrays()
    if window is not None:
        sel = (t >= window[0]) & (t <= window[1])
        tau, err, qd = tau[sel], err[sel], qd[sel]
    out = {}
    for i, j in enumerate(ARM_JOINTS):
        st = stall_nm(j)
        out[j] = {
            "servo": SERVO_OF[j],
            "tau_peak_Nm": round(float(np.abs(tau[:, i]).max()), 3),
            "tau_peak_pct_stall": round(float(np.abs(tau[:, i]).max() / st * 100), 1),
            "tau_rms_pct_stall": round(float(np.sqrt((tau[:, i] ** 2).mean()) / st * 100), 1),
            "saturated_pct_time": round(float((np.abs(tau[:, i]) > 0.98 * st).mean() * 100), 1),
            "err_max_deg": round(float(np.degrees(np.abs(err[:, i]).max())), 2),
            "err_rms_deg": round(float(np.degrees(np.sqrt((err[:, i] ** 2).mean()))), 2),
            "speed_peak_pct_noload": round(float(np.abs(qd[:, i]).max() / NO_LOAD_SPEED[SERVO_OF[j]] * 100), 1),
        }
    return out


# =======================================================================================
# A. Holding at full horizontal reach
# =======================================================================================
def scenario_hold():
    res = {}
    for payload in (0, 150, 250):
        m, d = build(payload)
        home = key_targets(m, "home")
        horiz = dict(home, shoulder=math.radians(90), gripper_pinion=key_targets(m, "rest")["gripper_pinion"])
        set_pose(m, d, home)
        rec = Recorder(m)
        run_profile(m, d, lambda t: {j: ease(home[j], horiz[j], t / 3.0) for j in ARM_JOINTS}, 6.0, rec)
        s = summarize(rec, window=(4.5, 6.0))
        sag = s["shoulder"]["err_max_deg"]
        res[f"{payload}g"] = {
            "shoulder_hold_pct_stall": s["shoulder"]["tau_peak_pct_stall"],
            "elbow_hold_pct_stall": s["elbow"]["tau_peak_pct_stall"],
            "wrist_hold_pct_stall": s["wrist_pitch"]["tau_peak_pct_stall"],
            "shoulder_sag_deg": sag, "elbow_sag_deg": s["elbow"]["err_max_deg"],
            "tip_sag_mm": round(math.radians(sag) * 420 + math.radians(s["elbow"]["err_max_deg"]) * 300, 1),
            "verdict": "PASS" if max(s[j]["tau_peak_pct_stall"] for j in ("shoulder", "elbow", "wrist_pitch")) < 98 else "FAIL (servo saturated, arm sags)",
        }
    return res


# =======================================================================================
# B. Demo motion (with 150 g payload)
# =======================================================================================
def demo_target_fn(m):
    keys = [(t, key_targets(m, k)) for t, k in DEMO]

    def fn(t):
        for (t0, a), (t1, b) in zip(keys, keys[1:]):
            if t <= t1:
                return {j: ease(a[j], b[j], (t - t0) / (t1 - t0)) for j in ARM_JOINTS}
        return keys[-1][1]
    return fn


def scenario_demo(payload=150):
    m, d = build(payload)
    fn = demo_target_fn(m)
    set_pose(m, d, fn(0))
    rec, contacts = Recorder(m), []
    run_profile(m, d, fn, DEMO[-1][0] + 1.0, rec, contacts=contacts)
    s = summarize(rec)
    pairs = sorted({(a, b) for a, b, _ in contacts})
    worst = max(v["tau_peak_pct_stall"] for v in s.values())
    return {"payload_g": payload, "joints": s, "collisions": [list(p) for p in pairs],
            "verdict": ("PASS" if worst < 98 and not pairs else
                        "FAIL" if pairs else "MARGINAL (one servo briefly saturates)")}, rec


# =======================================================================================
# C. Grasp and lift (with contact and friction)
# =======================================================================================
def ik(m, d, site, pos, z_down=True, iters=600, tilt_deg=0.0):
    """Damped least-squares IK: TCP position + gripper axis pointing down (tilted outward by
    tilt_deg), 5 joints, within the servo command range (ctrlrange)."""
    q, v, a = ids(m)
    joints = ["base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"]
    sid = m.site(site).id
    radial = np.array([pos[0], pos[1], 0.0])
    radial = radial / (np.linalg.norm(radial) or 1)
    t = math.radians(tilt_deg)
    want = np.array([0, 0, -math.cos(t)]) + radial * math.sin(t)     # tool axis: down, tilted outward
    for _ in range(iters):
        mujoco.mj_forward(m, d)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        mujoco.mj_jacSite(m, d, jp, jr, sid)
        e_p = np.array(pos) - d.site_xpos[sid]
        z = d.site_xmat[sid].reshape(3, 3)[:, 2]
        e_r = np.cross(z, want) if z_down else np.zeros(3)
        if np.linalg.norm(e_p) < 2e-4 and np.linalg.norm(e_r) < 2e-3:
            break
        e = np.concatenate([e_p, 0.3 * e_r])
        cols = [v[j] for j in joints]
        J = np.vstack([jp[:, cols], 0.3 * jr[:, cols]])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), e)
        dq = np.clip(dq, -0.05, 0.05)
        for k, j in enumerate(joints):
            lo, hi = m.actuator_ctrlrange[a[j]]
            d.qpos[q[j]] = float(np.clip(d.qpos[q[j]] + dq[k], lo, hi))
    mujoco.mj_forward(m, d)
    z = d.site_xmat[sid].reshape(3, 3)[:, 2]
    err = float(np.linalg.norm(np.array(pos) - d.site_xpos[sid]))
    ang = float(np.degrees(np.arccos(np.clip(np.dot(z, want), -1, 1))))
    return {j: float(d.qpos[q[j]]) for j in joints}, err + (0 if ang < 1.0 else 1.0)


def ik_best(m, seeds, pos, tilts=(0, 5, 10, 15, 20, 25, 30, 35)):
    """IK solved with the smallest tilt (from several seeds)."""
    for tilt in tilts:
        for seed in seeds:
            d = mujoco.MjData(m)
            set_pose(m, d, seed)
            sol, err = ik(m, d, "tcp", pos, tilt_deg=tilt)
            if err < 1e-3:
                return sol, err, tilt
    return sol, err, None


def ik_line(m, seed, start, end, tilt, n=12):
    """Cartesian straight line: IK at n waypoints between start->end with constant tilt (warm start)."""
    path, d = [], mujoco.MjData(m)
    set_pose(m, d, seed)
    for k in range(n + 1):
        p = np.array(start) + (np.array(end) - np.array(start)) * k / n
        sol, err = ik(m, d, "tcp", p, tilt_deg=tilt)
        if err > 1e-3:
            return None
        path.append(sol)
    return path


def scenario_grasp(obj_size=(0.03, 0.03, 0.06), obj_mass=100, obj_xy=(0.17, 0.0), hold=True):
    """Cartesian straight approach to a box on the table, close (MG90S at full torque), lift 120 mm,
    hold 2 s. Slip = displacement of the object relative to the TCP."""
    obj_pos = [obj_xy[0], obj_xy[1], obj_size[2] / 2 + 0.0005]
    m, d = build(0, obj=(obj_size, obj_mass, obj_pos))
    q, _, _ = ids(m)
    rest = key_targets(m, "rest")
    lo, hi = m.jnt_range[m.joint("gripper_pinion").id]
    g_open = hi - 0.01
    g_closed = key_targets(m, "grasp")["gripper_pinion"] if hold else lo   # gentle grip (width - 0.25 mm)
    center = np.array(obj_pos)
    grasp_p = center + [0, 0, 0.012]
    seeds = [rest, key_targets(m, "ready"), key_targets(m, "home")]
    down = up = None
    for tilt in (0, 5, 10, 15, 20, 25, 30, 35):
        sol, err, _ = ik_best(m, seeds, grasp_p, tilts=(tilt,))
        if err > 1e-3:
            continue
        seed = dict(rest, **sol)
        for adz in (0.06, 0.05, 0.04):
            down = ik_line(m, seed, grasp_p + [0, 0, adz], grasp_p, tilt)
            if down:
                break
        for dz in (0.12, 0.10, 0.08, 0.06):
            up = ik_line(m, seed, grasp_p, grasp_p + [0, 0, dz], tilt)
            if up:
                break
        if down and up:
            break
    if not down or not up:
        return {"verdict": "FAIL (IK: no straight approach found)"}
    approach_dz = adz

    def full(s, g):
        t = dict(rest)
        t.update(s)
        t["gripper_pinion"] = g
        return t

    wps = [(0.0, full(down[0], g_open))]
    t = 2.5
    wps.append((t, full(down[0], g_open)))
    for k, s_ in enumerate(down[1:], 1):
        wps.append((t + 1.5 * k / (len(down) - 1), full(s_, g_open)))
    t += 1.5
    wps.append((t + 1.0, full(down[-1], g_closed)))
    t += 1.5
    t_lift = t
    for k, s_ in enumerate(up[1:], 1):
        wps.append((t + 2.0 * k / (len(up) - 1), full(s_, g_closed)))
    t += 2.0
    wps.append((t + 2.0, full(up[-1], g_closed)))
    t_end = t + 2.0

    def fn(tt):
        for (t0, a_), (t1, b_) in zip(wps, wps[1:]):
            if tt <= t1:
                return {j: ease(a_[j], b_[j], (tt - t0) / (t1 - t0)) if t1 > t0 else b_[j] for j in ARM_JOINTS}
        return wps[-1][1]

    set_pose(m, d, full(rest, g_open))
    # start: first move from rest -> approach point (2.5 s)
    rest_open = full(rest, g_open)
    wps[0] = (0.0, rest_open)
    obj_q = m.jnt_qposadr[m.body("object").jntadr[0]]
    tcp = m.site("tcp").id
    rec, contacts, rel = Recorder(m), [], []
    obj_z0 = float(d.qpos[obj_q + 2])
    while d.time < t_end:
        tg = fn(d.time)
        for j in ARM_JOINTS:
            d.ctrl[m.actuator(f"servo_{j}").id] = tg[j]
        mujoco.mj_step(m, d)
        if int(d.time / 0.005) != int((d.time - m.opt.timestep) / 0.005):
            rec(d, tg)
            contacts.extend(bad_contacts(m, d))
            if d.time > t_lift:
                rel.append(d.site_xpos[tcp] - d.qpos[obj_q:obj_q + 3])
    f6 = np.zeros(6)
    normal = 0.0
    obj_geom = m.geom("object").id
    for i, c in enumerate(d.contact[:d.ncon]):
        other = c.geom2 if c.geom1 == obj_geom else c.geom1 if c.geom2 == obj_geom else None
        if other is not None and "pad" in (m.geom(other).name or ""):
            mujoco.mj_contactForce(m, d, i, f6)
            normal += abs(f6[0])
    normal /= 2
    lifted = float(d.qpos[obj_q + 2]) - obj_z0
    rel = np.array(rel)
    slip = float(np.linalg.norm(rel - rel[0], axis=1).max() * 1000) if len(rel) else float("nan")
    s = summarize(rec)
    return {
        "object": {"size_mm": [x * 1000 for x in obj_size], "mass_g": obj_mass, "at_mm": [x * 1000 for x in obj_pos]},
        "gripper_tilt_deg": tilt, "approach_mm": round(approach_dz * 1000), "lift_target_mm": round(dz * 1000),
        "lifted_mm": round(lifted * 1000, 1),
        "slip_during_lift_mm": round(slip, 2),
        "grip_force_per_finger_N": round(normal, 1),
        "required_per_finger_N_(mu=0.8,SF=1)": round(obj_mass / 1000 * 9.81 / (2 * 0.8), 2),
        "gripper_servo_pct_stall": s["gripper_pinion"]["tau_peak_pct_stall"],
        "arm_peak_pct_stall": {j: s[j]["tau_peak_pct_stall"] for j in ("shoulder", "elbow", "wrist_pitch")},
        "collisions": sorted({(a, b) for a, b, _ in contacts}),
        "verdict": "PASS" if lifted > 0.04 and slip < 3 else "FAIL",
    }


# =======================================================================================
# D. Thermal and electrical (from B's torque history)
# =======================================================================================
def thermal_electric(rec, cycles_min=30.0):
    t, tau, _, qd = rec.arrays()
    dt = np.diff(t, prepend=t[0])
    cur = {}
    for i, j in enumerate(ARM_JOINTS):
        sv = SERVO_OF[j]
        cur[j] = IDLE_CURRENT + STALL_CURRENT[sv] * np.abs(tau[:, i]) / stall_nm(j)
    rails = {r: sum(cur[j] for j in js) for r, js in RAIL.items()}
    period = t[-1]
    n = int(cycles_min * 60 / period)
    temps = {}
    for j in ARM_JOINTS:
        sv = SERVO_OF[j]
        r_w = SUPPLY_V / STALL_CURRENT[sv]
        p = cur[j] ** 2 * r_w
        T = T_AMB
        for _ in range(n):
            for k in range(len(t)):
                T += dt[k] * (p[k] - (T - T_AMB) / R_TH) / C_TH
        p_avg = float((p * dt).sum() / dt.sum())
        temps[j] = {"P_avg_W": round(p_avg, 2), f"T_after_{int(cycles_min)}min_C": round(T, 1),
                    "T_steady_C": round(T_AMB + p_avg * R_TH, 1)}
    hold = {}
    for j in ("shoulder", "elbow"):
        pass
    return {
        "rails": {r: {"peak_A": round(float(v.max()), 2), "avg_A": round(float((v * dt).sum() / dt.sum()), 2),
                      "verdict": "PASS" if float((v * dt).sum() / dt.sum()) < XL4016_CONT and v.max() < 8 else "FAIL"}
                  for r, v in rails.items()},
        "total_12V_input_A_peak": round(float(sum(rails.values()).max() * SUPPLY_V / 12 / 0.9), 2),
        "servo_temps": temps,
    }


def hold_thermal(hold_pct):
    """Steady-state temperature under continuous holding (e.g. horizontal reach)."""
    out = {}
    for j, pct in hold_pct.items():
        sv = SERVO_OF[j]
        i = IDLE_CURRENT + STALL_CURRENT[sv] * pct / 100
        p = i ** 2 * SUPPLY_V / STALL_CURRENT[sv]
        out[j] = {"I_A": round(i, 2), "P_W": round(p, 2), "T_steady_C": round(T_AMB + p * R_TH, 1),
                  "minutes_to_65C": (round(-R_TH * C_TH * math.log(1 - (T_LIMIT - T_AMB) / (p * R_TH)) / 60, 1)
                                     if p * R_TH > T_LIMIT - T_AMB else None)}
    return out


def plots(rec, name):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    t, tau, err, _ = rec.arrays()
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for i, j in enumerate(ARM_JOINTS):
        ax[0].plot(t, tau[:, i] / stall_nm(j) * 100, label=j)
        ax[1].plot(t, np.degrees(err[:, i]), label=j)
    ax[0].axhline(100, color="r", ls="--"); ax[0].axhline(-100, color="r", ls="--")
    ax[0].set_ylabel("torque / stall torque (%)"); ax[1].set_ylabel("tracking error (°)"); ax[1].set_xlabel("s")
    ax[0].legend(fontsize=7, ncol=3); ax[0].grid(alpha=.3); ax[1].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(OUT / f"{name}.png", dpi=110); plt.close(fig)


if __name__ == "__main__":
    results = {}
    results["A_hold_horizontal"] = scenario_hold()
    print("A", json.dumps(results["A_hold_horizontal"], indent=1))
    demo, rec = scenario_demo(150)
    results["B_demo_150g"] = demo
    plots(rec, "demo_150g")
    print("B", json.dumps(demo, indent=1))
    results["C_grasp_lift"] = scenario_grasp()
    print("C", json.dumps(results["C_grasp_lift"], indent=1, default=str))
    results["D_thermal_electric_demo_loop"] = thermal_electric(rec)
    h150 = results["A_hold_horizontal"]["150g"]
    results["D_thermal_hold_horizontal_150g"] = hold_thermal(
        {"shoulder": h150["shoulder_hold_pct_stall"], "elbow": h150["elbow_hold_pct_stall"]})
    print("D", json.dumps({k: results[k] for k in results if k.startswith("D")}, indent=1))
    (OUT / "dynamics.json").write_text(json.dumps(results, indent=1, default=str), encoding="utf-8")

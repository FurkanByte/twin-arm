"""TwinArm-5 digital-twin core: MuJoCo model + servo electrical/thermal model + ESP32 link.

Used by the web server (server.py). In sim mode, commands go to the MuJoCo model (with the real servo
torque limit, damping, friction and position control); current and temperature are computed with the
model in simulation/dynamics.py. In live mode, the angle/current/temperature sent by the ESP32 is used.

ESP32 -> PC (every 10 ms, one line of JSON):  {"q": [6 angles °], "i": [6 currents A], "T": [6 temperatures °C], "v": 12.1}
PC -> ESP32:  "J a1 a2 a3 a4 a5 a6\\n" (degrees; a6 = gripper pinion angle),  "S\\n" (software stop)
"""
import json
import math
import os
import queue
import threading
import time
from collections import deque

import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "..", "simulation", "models", "mujoco", "scene.xml")

JOINTS = ["base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper_pinion"]
NAMES = {"base_yaw": "Base", "shoulder": "Shoulder", "elbow": "Elbow",
         "wrist_pitch": "Wrist pitch", "wrist_roll": "Wrist roll", "gripper_pinion": "Gripper"}
SERVO = {"base_yaw": "DS3218MG", "shoulder": "RDS3235", "elbow": "DS3230MG",
         "wrist_pitch": "DS3218MG", "wrist_roll": "DS3218MG", "gripper_pinion": "MG90S"}
SERVO_GEOM = {"base_yaw": "body", "shoulder": "body_2", "elbow": "body_3", "wrist_pitch": "body_4",
              "wrist_roll": "body_5", "gripper_pinion": "body_6"}
# software limits (cad/arm.py RANGES) and gripper pinion 0..105°
RANGE = {"base_yaw": (-90, 90), "shoulder": (-70, 105), "elbow": (-60, 105), "wrist_pitch": (-60, 95),
         "wrist_roll": (-90, 90), "gripper_pinion": (0, 105)}

# --- electrical / thermal model (same as simulation/dynamics.py) -----------------------------
SUPPLY_V = 6.0
MM_PER_DEG = math.pi * 24.0 / 360.0      # pinion (m1.5 z16) pitch diameter 24 mm: mm / degree
STALL_CURRENT = {"DS3218MG": 2.5, "DS3230MG": 3.0, "RDS3235": 3.2, "MG90S": 0.7}   # A @6 V (estimate)
IDLE_CURRENT = 0.08
R_TH, C_TH, T_AMB = 14.0, 45.0, 25.0
T_WARN, T_LIMIT = 50.0, 65.0
PCT_WARN, PCT_CRIT = 60.0, 85.0
ALERT_ON, ALERT_HOLD = 0.4, 1.5          # s: an alert must persist to show, then lingers
RAIL = {"Rail 1 (J2, J4, J6)": ["shoulder", "wrist_pitch", "gripper_pinion"],
        "Rail 2 (J1, J3, J5)": ["base_yaw", "elbow", "wrist_roll"]}

DEMO = [(0, "rest"), (2, "ready"), (4, "pick_above"), (5.5, "pick"), (6.5, "grasp"),
        (8, "pick_above"), (10, "carry"), (12, "place_above"), (13.5, "place"), (14.5, "release"),
        (16, "place_above"), (18, "ready"), (20, "reach"), (22, "rest")]
POSES = {"home": "Upright", "rest": "Rest", "ready": "Ready", "pick": "Pick", "carry": "Carry",
         "place": "Place", "reach": "Reach"}


def ease(a, b, s):
    s = min(max(s, 0.0), 1.0)
    return a + (3 * s * s - 2 * s ** 3) * (b - a)


def level(pct, temp):
    if pct >= PCT_CRIT or temp >= T_LIMIT:
        return 2
    if pct >= PCT_WARN or temp >= T_WARN:
        return 1
    return 0


# ------------------------------------------------------------------------------------------
class Twin:
    """Model + servo state (torque, current, temperature)."""

    def __init__(self):
        spec = mujoco.MjSpec.from_file(SCENE)
        probe = spec.compile()
        sid = probe.site("tcp").id
        tcp_body = probe.body(probe.site_bodyid[sid]).name
        b = spec.body(tcp_body).add_body(name="payload", pos=probe.site_pos[sid].copy())
        b.mass, b.inertia, b.explicitinertial = 1e-6, [1e-9, 1e-9, 1e-9], True
        b.add_geom(name="payload", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.012, 0, 0],
                   rgba=[0.2, 0.6, 0.95, 1.0], contype=0, conaffinity=0, group=2, mass=0)
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.payload_body = m.body("payload").id
        self.qadr = {j: m.jnt_qposadr[m.joint(j).id] for j in JOINTS}
        self.vadr = {j: m.jnt_dofadr[m.joint(j).id] for j in JOINTS}
        self.act = {j: m.actuator(f"servo_{j}").id for j in JOINTS}
        self.stall = {j: float(m.actuator_forcerange[self.act[j], 1]) for j in JOINTS}
        self.geom = {j: m.geom(SERVO_GEOM[j]).id for j in JOINTS}
        self.keys = {m.key(k).name: k for k in range(m.nkey)}
        self.temp = {j: T_AMB for j in JOINTS}
        self.pct = {j: 0.0 for j in JOINTS}
        self.cur = {j: IDLE_CURRENT for j in JOINTS}
        self.expected = {j: 0.0 for j in JOINTS}
        self.p_avg = {j: 0.0 for j in JOINTS}      # W, ~5 s moving average (for the 65 °C forecast)
        self.reset("ready")

    def key_targets(self, name):
        k = self.keys[name]
        return {j: float(self.m.key_qpos[k][self.qadr[j]]) for j in JOINTS}

    def reset(self, pose):
        mujoco.mj_resetDataKeyframe(self.m, self.d, self.keys[pose])
        for j, v in self.key_targets(pose).items():
            self.d.ctrl[self.act[j]] = v
        mujoco.mj_forward(self.m, self.d)

    def set_payload(self, grams):
        m = self.m
        kg = max(grams, 1e-3) / 1000
        m.body_mass[self.payload_body] = kg
        m.body_inertia[self.payload_body] = [0.4 * kg * 0.012 ** 2] * 3
        mujoco.mj_setConst(m, self.d)

    def q_deg(self, j):
        return math.degrees(self.d.qpos[self.qadr[j]])

    def step_sim(self, dt, targets):
        """Moves toward the target with linear interpolation within the step (continuous, like a servo command)."""
        n = max(1, int(round(dt / self.m.opt.timestep)))
        start = {j: float(self.d.ctrl[self.act[j]]) for j in JOINTS}
        goal = {}
        for j, v in targets.items():
            lo, hi = self.m.actuator_ctrlrange[self.act[j]]
            goal[j] = min(max(v, lo), hi)
        tau = {j: 0.0 for j in JOINTS}
        for k in range(n):
            a = (k + 1) / n
            for j, g in goal.items():
                self.d.ctrl[self.act[j]] = start[j] + a * (g - start[j])
            mujoco.mj_step(self.m, self.d)
            for j in JOINTS:
                tau[j] = max(tau[j], abs(float(self.d.actuator_force[self.act[j]])))
        for j in JOINTS:
            self.pct[j] = 100.0 * tau[j] / self.stall[j]
            self.cur[j] = IDLE_CURRENT + STALL_CURRENT[SERVO[j]] * self.pct[j] / 100.0
            # torque gravity alone needs at this pose (bias force; velocities are small here)
            self.expected[j] = 100.0 * abs(float(self.d.qfrc_bias[self.vadr[j]])) / self.stall[j]

    def apply_measured(self, q_deg, cur, temp):
        for j, a in zip(JOINTS, q_deg):
            self.d.qpos[self.qadr[j]] = math.radians(a)
        for side in ("finger_left", "finger_right"):        # fingers are driven by the pinion
            self.d.qpos[self.m.jnt_qposadr[self.m.joint(side).id]] = q_deg[5] * MM_PER_DEG / 1000.0
        self.d.qvel[:] = 0
        mujoco.mj_forward(self.m, self.d)                  # qfrc_bias: gravity torque in this pose
        for k, j in enumerate(JOINTS):
            self.expected[j] = 100.0 * abs(float(self.d.qfrc_bias[self.vadr[j]])) / self.stall[j]
            if cur is not None:
                self.cur[j] = float(cur[k])
                self.pct[j] = 100.0 * max(0.0, self.cur[j] - IDLE_CURRENT) / STALL_CURRENT[SERVO[j]]
            if temp is not None:
                self.temp[j] = float(temp[k])

    def thermal(self, dt):
        for j in JOINTS:
            p = self.cur[j] ** 2 * SUPPLY_V / STALL_CURRENT[SERVO[j]]
            self.temp[j] += dt * (p - (self.temp[j] - T_AMB) / R_TH) / C_TH

    def update_power(self, dt, tau=5.0):
        """Smooth the electrical power so the time-to-limit forecast does not jump with every move."""
        k = min(1.0, dt / tau)
        for j in JOINTS:
            p = self.cur[j] ** 2 * SUPPLY_V / STALL_CURRENT[SERVO[j]]
            self.p_avg[j] += (p - self.p_avg[j]) * k

    def minutes_to_limit(self, j):
        """Minutes until 65 °C if this power is held (first-order model). None: never reached."""
        p = self.p_avg[j]
        t_inf = T_AMB + p * R_TH
        T = self.temp[j]
        if T >= T_LIMIT:
            return 0.0
        if t_inf <= T_LIMIT:
            return None
        return -R_TH * C_TH * math.log((t_inf - T_LIMIT) / (t_inf - T)) / 60.0


# ------------------------------------------------------------------------------------------
class Serial(threading.Thread):
    def __init__(self, port, baud=921600):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.q = queue.Queue(maxsize=4)
        self.alive = True
        self.last_rx = 0.0

    def run(self):
        buf = b""
        while self.alive:
            try:
                buf += self.ser.read(512)
            except Exception:
                break
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line.decode(errors="ignore"))
                except ValueError:
                    continue
                self.last_rx = time.time()
                if self.q.full():
                    self.q.get_nowait()
                self.q.put(msg)

    def write(self, text):
        self.ser.write(text.encode())

    def close(self):
        self.alive = False
        try:
            self.ser.close()
        except Exception:
            pass


def list_ports():
    try:
        from serial.tools import list_ports as lp
        return [{"device": p.device, "desc": p.description} for p in lp.comports()]
    except Exception:
        return []


# ------------------------------------------------------------------------------------------
# Motion shaping + software torque limiter
# ------------------------------------------------------------------------------------------
KP_ERR_DEG = 3.0          # servo model (simulation/mujoco_model.py): stall torque is reached at 3° position error
VMAX = {"base_yaw": 90.0, "shoulder": 60.0, "elbow": 70.0, "wrist_pitch": 90.0, "wrist_roll": 120.0,
        "gripper_pinion": 150.0}                                   # deg/s, well under no-load speed
AMAX = {"base_yaw": 240.0, "shoulder": 150.0, "elbow": 180.0, "wrist_pitch": 240.0, "wrist_roll": 360.0,
        "gripper_pinion": 600.0}                                   # deg/s²
TORQUE_LIMITED = [j for j in JOINTS if j != "gripper_pinion"]      # the gripper squeezes on purpose
BLOCK_TIME = 1.0          # s at the torque limit without progress -> the motion is stopped


class MotionLimiter:
    """Turns a jump in the goal into a smooth, speed- and acceleration-limited move and keeps every
    arm servo below a torque limit.

    1. Profile: each joint follows a velocity/acceleration-limited path to its goal. For a pose
       command the joints are time-synchronised, so they start and arrive together.
    2. Torque clamp: a position servo's effort grows with the error between command and actual angle
       (stall at KP_ERR_DEG). The command sent to a servo is kept within limit% of that error from
       the measured angle, so the servo never asks for more than limit% of its stall torque.
    3. Governor: while any arm servo works near the limit, the whole motion slows down; it speeds
       back up when the load drops. Works the same on the real arm (angle from the potentiometer).
    """

    def __init__(self, start_deg, limit_pct=80.0):
        self.enabled = True
        self.limit_pct = limit_pct
        self.reset(start_deg)

    def reset(self, start_deg):
        self.sp = dict(start_deg)                # profiled setpoint (deg)
        self.v = {j: 0.0 for j in JOINTS}        # deg/s
        self.goal = dict(start_deg)
        self.vlim = dict(VMAX)
        self.speed = 1.0                         # governor factor 0.1 .. 1
        self.clamped = set()
        self.cmd = dict(start_deg)
        self.stuck_t = {j: 0.0 for j in JOINTS}  # time spent at the limit without getting closer
        self.best_err = {j: 0.0 for j in JOINTS}
        self.blocked = []

    def set_goal(self, goal_deg, sync=False):
        moved = {j for j in JOINTS if abs(goal_deg[j] - self.goal[j]) > 1e-6}
        self.goal = dict(goal_deg)
        if not moved:
            return
        if sync:                                 # pose command: all joints arrive together
            t = max(abs(self.goal[j] - self.sp[j]) / VMAX[j] for j in JOINTS)
            for j in JOINTS:
                d = abs(self.goal[j] - self.sp[j])
                self.vlim[j] = VMAX[j] if t < 1e-6 else min(VMAX[j], max(d / t, 2.0))
        else:
            for j in moved:
                self.vlim[j] = VMAX[j]

    def budget(self, grav):
        """Torque budget per joint (% of stall): the limit, or what gravity needs plus 10 %,
        whichever is higher (never above 98 %), so a heavy load can still be held and lifted."""
        return {j: min(98.0, max(self.limit_pct, grav.get(j, 0.0) + 10.0)) for j in JOINTS}

    def step(self, dt, q_deg, pct, grav=None):
        b = self.budget(grav or {})
        self.last_budget = b
        if self.enabled:
            worst = max(pct[j] - b[j] for j in TORQUE_LIMITED) + self.limit_pct
            if worst > self.limit_pct - 5:
                self.speed = max(0.1, self.speed - 3.0 * dt)
            elif worst < self.limit_pct - 20:
                self.speed = min(1.0, self.speed + 0.6 * dt)
        else:
            self.speed = 1.0
        g = self.speed
        for j in JOINTS:
            err = self.goal[j] - self.sp[j]
            a = AMAX[j] * g
            v_des = math.copysign(min(self.vlim[j] * g, math.sqrt(2.0 * a * abs(err))), err)
            dv = max(-a * dt, min(a * dt, v_des - self.v[j]))
            self.v[j] += dv
            nxt = self.sp[j] + self.v[j] * dt
            if (self.goal[j] - nxt) * err <= 0:  # reached or passed the goal
                nxt, self.v[j] = self.goal[j], 0.0
            self.sp[j] = nxt
        cmd = dict(self.sp)
        self.clamped = set()
        if self.enabled:
            for j in TORQUE_LIMITED:
                e = b[j] / 100.0 * KP_ERR_DEG
                lo, hi = q_deg[j] - e, q_deg[j] + e
                if not lo <= cmd[j] <= hi:
                    cmd[j] = min(max(cmd[j], lo), hi)
                    self.sp[j] = cmd[j]          # the profile waits for the servo to catch up
                    self.v[j] *= 0.5
                    self.clamped.add(j)
        # jam / collision: at the torque limit for BLOCK_TIME s without getting closer to the goal
        self.blocked = []
        for j in TORQUE_LIMITED:
            err = abs(self.goal[j] - q_deg[j])
            if j in self.clamped and err > 1.5 and err >= self.best_err[j] - 0.3:
                self.stuck_t[j] += dt
            else:
                self.stuck_t[j] = 0.0
                self.best_err[j] = err
            self.best_err[j] = min(self.best_err[j], err)
            if self.stuck_t[j] >= BLOCK_TIME:
                self.blocked.append(j)
        self.cmd = cmd
        return cmd

    def hold(self, q_deg):
        self.reset(q_deg)

    @property
    def moving(self):
        return any(abs(self.goal[j] - self.sp[j]) > 0.05 or abs(self.v[j]) > 0.05 for j in JOINTS)


# ------------------------------------------------------------------------------------------
class Controller:
    """Session state + 100 Hz simulation/telemetry loop. All access goes through the lock."""

    def __init__(self, port=None):
        self.twin = Twin()
        self.lock = threading.Lock()
        self.mode = "sim"                     # "sim" | "live"
        self.targets = self.twin.key_targets("ready")
        self.demo_t = None
        self.payload = 0
        self.warp = 1
        self.stopped = False
        self.serial = None
        self.port = ""
        self.vin = None
        self.events = deque(maxlen=80)
        self._alert_state = {}               # key -> first/last seen, shown (debounced alerts)
        self.message = ""
        self.lim = MotionLimiter(self._q())
        self._tx = 0
        self._live_sync = True
        if port:
            self.connect(port)
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _q(self):
        return {j: self.twin.q_deg(j) for j in JOINTS}

    def _goal(self, sync=False):
        self.lim.set_goal({j: math.degrees(v) for j, v in self.targets.items()}, sync=sync)

    def _reset_motion(self):
        self.lim.reset(self._q())
        self.targets = {j: math.radians(v) for j, v in self._q().items()}

    def _halt_blocked(self):
        names = ", ".join(NAMES[j] for j in self.lim.blocked)
        self.demo_t = None
        self._reset_motion()
        self._send_targets()
        text = f"Motion stopped: {names} at the torque limit without progress (collision or jam?)"
        self.message = text
        self._event(2, text)

    # --- loop --------------------------------------------------------------------------------
    def _loop(self):
        last = time.perf_counter()
        while True:
            time.sleep(0.01)
            now = time.perf_counter()
            dt = min(now - last, 0.05)
            last = now
            with self.lock:
                tw = self.twin
                if self.mode == "sim":
                    if self.demo_t is not None and not self.stopped:
                        self.demo_t += dt
                        self.targets = self._demo_targets(self.demo_t)
                        self._goal()
                    cmd = self.lim.step(dt, self._q(), tw.pct, tw.expected)
                    tw.step_sim(dt, {j: math.radians(v) for j, v in cmd.items()})
                    tw.thermal(dt * self.warp)
                else:
                    msg = None
                    while self.serial and not self.serial.q.empty():
                        msg = self.serial.q.get_nowait()
                    if msg:
                        q = msg.get("q") or [tw.q_deg(j) for j in JOINTS]
                        tw.apply_measured(q, msg.get("i"), msg.get("T"))
                        if self._live_sync:              # first data after connecting: hold where the arm is
                            self._reset_motion()
                            self._live_sync = False
                        self.vin = msg.get("v")
                        if msg.get("T") is None:
                            tw.thermal(dt * self.warp)
                    if self.demo_t is not None and not self.stopped:
                        self.demo_t += dt
                        self.targets = self._demo_targets(self.demo_t)
                        self._goal()
                    was_moving = self.lim.moving
                    self.lim.step(dt, self._q(), tw.pct, tw.expected)
                    self._tx += 1
                    if (was_moving or self.lim.clamped) and self._tx % 2 == 0:     # 50 Hz while moving
                        self._send_targets()
                if self.lim.blocked and not self.stopped:
                    self._halt_blocked()
                tw.update_power(dt * self.warp)
                self._update_events()

    def _demo_targets(self, t):
        t = t % DEMO[-1][0]
        for (t0, p0), (t1, p1) in zip(DEMO, DEMO[1:]):
            if t0 <= t <= t1:
                a, b = self.twin.key_targets(p0), self.twin.key_targets(p1)
                s = (t - t0) / (t1 - t0)
                return {j: ease(a[j], b[j], s) for j in JOINTS}
        return self.twin.key_targets(DEMO[0][1])

    def _send_targets(self):
        if self.serial:
            try:
                self.serial.write("J " + " ".join(f"{self.lim.cmd[j]:.1f}" for j in JOINTS) + "\n")
            except Exception as e:
                self.message = f"Could not send to ESP32: {e}"

    # --- commands (from the web UI) --------------------------------------------------------
    def command(self, c):
        kind = c.get("cmd")
        with self.lock:
            if kind == "stop":
                self.stopped = True
                self.demo_t = None
                self._reset_motion()
                if self.serial:
                    try:
                        self.serial.write("S\n")
                    except Exception:
                        pass
                self._event(2, "Stop command issued: arm is held in place")
                return
            if kind == "resume":
                self.stopped = False
                self._event(0, "Stop cleared")
                return
            if kind == "payload":
                self.payload = int(c.get("g", 0))
                self.twin.set_payload(self.payload)
                return
            if kind == "warp":
                self.warp = max(1, min(int(c.get("x", 1)), 600))
                return
            if kind == "limiter":
                if "on" in c:
                    self.lim.enabled = bool(c["on"])
                if "pct" in c:
                    self.lim.limit_pct = max(50.0, min(float(c["pct"]), 100.0))
                self._event(0, f"Torque limiter {'on' if self.lim.enabled else 'off'}"
                               f" ({self.lim.limit_pct:.0f}% of stall)")
                return
            if kind == "cool":
                for j in JOINTS:
                    self.twin.temp[j] = T_AMB
                return
            if kind == "mode":
                self._set_mode(c.get("mode", "sim"), c.get("port"))
                return
            if self.stopped:
                self.message = "Stop is active: press 'Resume' before moving"
                return
            if kind == "pose" and c.get("name") in self.twin.keys:
                self.demo_t = None
                self.targets = self.twin.key_targets(c["name"])
                self._goal(sync=True)
                self.message = ""

            elif kind == "demo":
                if c.get("on"):
                    if self.mode == "sim":
                        self.twin.reset(DEMO[0][1])
                        self._reset_motion()
                    self.demo_t = 0.0
                else:
                    self.demo_t = None
            elif kind == "target" and c.get("joint") in JOINTS:
                j = c["joint"]
                lo, hi = RANGE[j]
                self.demo_t = None
                self.targets[j] = math.radians(min(max(float(c.get("deg", 0)), lo), hi))
                self._goal()
                self.message = ""

    def _set_mode(self, mode, port=None):
        if mode == "live":
            if port:
                self.connect(port)
            if not self.serial:
                self.message = "Select a serial port and connect for live mode"
                return
            self.mode = "live"
            self.demo_t = None
        else:
            if self.serial:
                self.serial.close()
                self.serial = None
            self.mode = "sim"
            self.twin.reset("ready")
            self._reset_motion()
            self.demo_t = None
            self.message = ""

    def connect(self, port):
        if self.serial:
            self.serial.close()
            self.serial = None
        try:
            self.serial = Serial(port)
            self.serial.start()
            self.port = port
            self.mode = "live"
            self._live_sync = True
            self.message = f"{port} connected"
            self._event(0, f"ESP32 connected: {port}")
        except Exception as e:
            self.serial = None
            self.message = f"Could not open {port}: {e}"
            self._event(1, f"Connection failed: {port}")

    # --- alerts and event log ---------------------------------------------------------------
    def _event(self, lvl, text):
        self.events.appendleft({"t": time.strftime("%H:%M:%S"), "lvl": lvl, "text": text})

    def alerts(self):
        tw = self.twin
        out = []
        for j in JOINTS:
            pct, T = tw.pct[j], tw.temp[j]
            mins = tw.minutes_to_limit(j)
            if pct >= PCT_CRIT:
                out.append((2, f"{j}:tq", f"{NAMES[j]}: torque {pct:.0f}%, servo near saturation"))
            elif pct >= PCT_WARN:
                out.append((1, f"{j}:tq", f"{NAMES[j]} under strain: torque {pct:.0f}%"))
            if T >= T_LIMIT:
                out.append((2, f"{j}:T", f"{NAMES[j]} {T:.0f} °C: at thermal limit, drop the load and let it rest"))
            elif mins is not None and mins < 15:
                out.append((1, f"{j}:T", f"{NAMES[j]}: reaches 65 °C in {mins:.1f} min at this load"))
            if self.mode == "live" and pct - tw.expected[j] > 25:
                out.append((1, f"{j}:x", f"{NAMES[j]}: measured torque {pct - tw.expected[j]:.0f}% above model "
                                         "(jam, collision or friction?)"))
        if self.mode == "live" and self.serial and time.time() - self.serial.last_rx > 1.0:
            out.append((2, "link", "No data from ESP32 for 1 second"))
        return out

    def _update_events(self):
        """Debounce alerts: one appears only after its condition has held for ALERT_ON seconds and
        stays ALERT_HOLD seconds after it clears, so short torque spikes while moving between poses
        do not make the warning list (and the page layout) flicker."""
        now = time.time()
        for lvl, k, text in self.alerts():
            s = self._alert_state.get(k)
            if s is None:
                s = self._alert_state[k] = {"first": now, "shown": False}
            s.update(last=now, lvl=lvl, text=text)
            if not s["shown"] and now - s["first"] >= ALERT_ON:
                s["shown"] = True
                if lvl >= 1:
                    self._event(lvl, text)
        for k in [k for k, s in self._alert_state.items()
                  if now - s["last"] > (ALERT_HOLD if s["shown"] else 0.05)]:
            del self._alert_state[k]

    def visible_alerts(self):
        return sorted(((s["lvl"], k, s["text"]) for k, s in self._alert_state.items() if s["shown"]),
                      key=lambda a: -a[0])

    # --- state (broadcast to web) -------------------------------------------------------------
    def snapshot(self):
        with self.lock:
            tw, m, d = self.twin, self.twin.m, self.twin.d
            bodies = [[round(float(v), 5) for v in (*d.xpos[b], *d.xquat[b])] for b in range(m.nbody)]
            joints = {}
            for j in JOINTS:
                mins = tw.minutes_to_limit(j)
                joints[j] = {"cmd": round(math.degrees(self.targets[j]), 1), "q": round(tw.q_deg(j), 1),
                             "pct": round(tw.pct[j], 1), "cur": round(tw.cur[j], 3),
                             "temp": round(tw.temp[j], 1), "ttl": None if mins is None else round(mins, 1),
                             "exp": round(tw.expected[j], 1), "lvl": level(tw.pct[j], tw.temp[j])}
            rails = {r: round(sum(tw.cur[j] for j in js), 2) for r, js in RAIL.items()}
            alerts = [{"lvl": lvl, "text": text} for lvl, _, text in self.visible_alerts()]
            return {
                "type": "state", "mode": self.mode, "port": self.port if self.serial else "",
                "simTime": round(float(d.time), 2), "demo": None if self.demo_t is None else round(self.demo_t % DEMO[-1][0], 2),
                "demoLen": DEMO[-1][0], "payload": self.payload, "warp": self.warp, "stopped": self.stopped,
                "bodies": bodies, "joints": joints, "rails": rails,
                "input12": round(sum(rails.values()) * SUPPLY_V / 12 / 0.9, 2), "vin": self.vin,
                "limiter": {"on": self.lim.enabled, "pct": self.lim.limit_pct, "speed": round(self.lim.speed, 2),
                            "clamped": [NAMES[j] for j in sorted(self.lim.clamped)], "moving": self.lim.moving},
                "alerts": alerts, "events": list(self.events)[:30], "message": self.message,
            }

    def model_info(self):
        """For the browser to build the 3D scene: bodies, visual geoms, servo mapping."""
        m = self.twin.m
        servo_of = {g: j for j, g in self.twin.geom.items()}
        geoms = []
        for g in range(m.ngeom):
            if m.geom_group[g] != 2:
                continue
            t = m.geom_type[g]
            if t not in (mujoco.mjtGeom.mjGEOM_MESH, mujoco.mjtGeom.mjGEOM_SPHERE):
                continue
            mat = m.geom_matid[g]
            rgba = m.mat_rgba[mat] if mat >= 0 else m.geom_rgba[g]
            mname = m.material(mat).name if mat >= 0 else ""
            geoms.append({
                "id": g, "name": m.geom(g).name, "body": int(m.geom_bodyid[g]),
                "type": "mesh" if t == mujoco.mjtGeom.mjGEOM_MESH else "sphere",
                "mesh": int(m.geom_dataid[g]), "size": [float(v) for v in m.geom_size[g]],
                "pos": [float(v) for v in m.geom_pos[g]], "quat": [float(v) for v in m.geom_quat[g]],
                "rgba": [round(float(v), 4) for v in rgba],
                "metal": any(k in mname for k in ("Steel", "Alu", "Brass")),
                "servo": servo_of.get(g), "payload": int(m.geom_bodyid[g]) == self.twin.payload_body,
            })
        return {
            "bodies": [m.body(b).name for b in range(m.nbody)], "geoms": geoms, "nmesh": int(m.nmesh),
            "joints": [{"id": j, "name": NAMES[j], "servo": SERVO[j], "range": RANGE[j],
                        "stall": round(self.twin.stall[j], 3)} for j in JOINTS],
            "poses": [{"id": k, "name": v} for k, v in POSES.items()],
            "limits": {"tWarn": T_WARN, "tLimit": T_LIMIT, "pctWarn": PCT_WARN, "pctCrit": PCT_CRIT, "tAmb": T_AMB},
        }

    def mesh_bytes(self, i):
        m = self.twin.m
        va, vn = m.mesh_vertadr[i], m.mesh_vertnum[i]
        fa, fn = m.mesh_faceadr[i], m.mesh_facenum[i]
        v = m.mesh_vert[va:va + vn].astype("<f4")
        f = m.mesh_face[fa:fa + fn].astype("<u4")
        import struct
        return struct.pack("<II", vn, fn) + v.tobytes() + f.tobytes()

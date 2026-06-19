import os
import math
import yaml
import numpy as np
import casadi as ca
import rospy

from crs_msgs.msg import car_state_cart, car_input
from visualization_msgs.msg import Marker

from rospy_controllers.rospy_controller import RosPyController
from rospy_controllers.utils import unwrap_yaw_angle, get_closest_track_point_idx, check_file_changes
from .generate_solver import build_solver

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_WORKDIR  = os.path.abspath(os.path.join(_THIS_DIR, "../../../"))  # rospy_controllers root


class MPCCPacejkaController(RosPyController):
    """
    Python MPCC controller for the Pacejka car model.

    Mirrors C++ PacejkaMpccController<AcadosPacejkaMpccSolver> exactly:
      - EXTERNAL cost
      - SQP-RTI (1 iteration per tick by default)
      - Lag compensation via RK4 forward simulation
      - Warmstart on first call
    """

    # State indices
    IX, IY, IYAW, IVX, IVY, IDYAW, IT, IDELTA, ITHETA = range(9)
    # Input indices
    IDT, IDDELTA, IDTHETA = range(3)

    def __init__(self, track: dict, pacejka_params: dict, controller_params: dict):
        self._parse_track(track)
        self._parse_params(pacejka_params, controller_params)
        self._build_lag_compensator()
        self._setup_solver()

        self._theta = 0.0
        self._laps  = 0
        self._last_torque = 0.5
        self._last_steer  = 0.0
        self._last_x = np.zeros((self.N + 1, 9))
        self._last_u = np.zeros((self.N, 3))
        self._last_ref = np.zeros((self.N, 3))  # [xd, yd, phi] per stage
        self._initialized = False

        self._vis_pub = rospy.Publisher(
            "~visualizer/controller_visualization", Marker, queue_size=3 * self.N
        )
        self._vis_params = rospy.get_param("~visualizer", {})

    # ── Initialisation helpers ─────────────────────────────────────────────────

    def _parse_track(self, track: dict):
        self._track_x    = np.array(track["xCoords"])
        self._track_y    = np.array(track["yCoords"])
        self._track_arc  = np.array(track["arcLength"])
        self._track_xrate = np.array(track["xRate"])
        self._track_yrate = np.array(track["yRate"])
        self._track_density   = float(track["density"])
        self._track_N         = len(self._track_x)
        self._track_max_arc   = float(np.max(self._track_arc))
        self._track_phi       = unwrap_yaw_angle(track["tangentAngle"])

    def _parse_params(self, pacejka_params: dict, controller_params: dict):
        mp = pacejka_params["model_params"]
        # 19 Pacejka model params (order matches the parameter vector in pacejka_model.py)
        self._model_params = [
            mp["lr"], mp["lf"], mp["m"], mp["I"],
            mp["Df"], mp["Cf"], mp["Bf"],
            mp["Dr"], mp["Cr"], mp["Br"],
            mp["Cm1"], mp["Cm2"],
            mp["Cd0"], mp["Cd1"], mp["Cd2"],
            mp["gamma"], mp["eps"],
            mp["car_width"], mp["wheel_radius"],
        ]
        # 6 cost weights
        self._cost_params = [
            controller_params["Q1"], controller_params["Q2"],
            controller_params["R1"], controller_params["R2"],
            controller_params["R3"], controller_params["q"],
        ]
        self._lag_dt          = float(controller_params.get("lag_compensation_time", 0.02))
        self._warmstart_iters = int(controller_params.get("warmstart_iterations", 50))
        self._max_sqp_iter    = int(controller_params.get("max_sqp_iterations", 1))

        # First 17 model params are used by the dynamics (no car_width/wheel_radius)
        self._dyn_params = np.array(self._model_params[:17])

    def _build_lag_compensator(self):
        """Build a CasADi function for the 6-DOF Pacejka dynamics (lag compensation)."""
        x6   = ca.SX.sym("x6", 6)     # [x, y, yaw, vx, vy, omega]
        T_s  = ca.SX.sym("T")
        d_s  = ca.SX.sym("delta")
        p    = ca.SX.sym("p", 17)     # [lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br,
                                      #  Cm1, Cm2, Cd0, Cd1, Cd2, gamma, eps]

        x, y, yaw, vx, vy, omega = [x6[i] for i in range(6)]
        lr, lf, m, I = p[0], p[1], p[2], p[3]
        Df, Cf, Bf   = p[4], p[5], p[6]
        Dr, Cr, Br   = p[7], p[8], p[9]
        Cm1, Cm2     = p[10], p[11]
        Cd0, Cd1, Cd2 = p[12], p[13], p[14]
        gamma, eps   = p[15], p[16]

        w_r = -vy + lr * omega
        b_r = 0.5*w_r/(eps**2 + w_r**2) + 3/(2*eps)*ca.atan(w_r/eps)
        c_r = -1/(2*eps**3)*ca.atan(w_r/eps) - w_r/(2*eps**2)/(w_r**2 + eps**2)
        g_r = b_r*vx + c_r*vx**3

        w_f = -vy - lf * omega
        b_f = 0.5*w_f/(eps**2 + w_f**2) + 3/(2*eps)*ca.atan(w_f/eps)
        c_f = -1/(2*eps**3)*ca.atan(w_f/eps) - w_f/(2*eps**2)/(w_f**2 + eps**2)
        g_f = b_f*vx + c_f*vx**3

        ar = ca.if_else(vx < eps, g_r, ca.atan2(-vy + lr*omega, vx))
        af = ca.if_else(vx < eps, g_f, d_s + ca.atan2(-vy - lf*omega, vx))

        Fm        = (Cm1 - Cm2*vx) * T_s
        Ffriction = ca.sign(vx) * (-Cd0 - Cd1*vx - Cd2*vx**2)
        Fx_f = Fm * (1 - gamma)
        Fx_r = Fm * gamma
        Fy_f = Df * ca.sin(Cf * ca.atan(Bf * af))
        Fy_r = Dr * ca.sin(Cr * ca.atan(Br * ar))

        Fx = Fx_r + Fx_f*ca.cos(d_s) - Fy_f*ca.sin(d_s) + m*vy*omega + Ffriction
        Fy = Fy_r + Fx_f*ca.sin(d_s) + Fy_f*ca.cos(d_s) - m*vx*omega
        Mz = Fy_f*lf*ca.cos(d_s) + Fx_f*lf*ca.sin(d_s) - Fy_r*lr

        f6 = ca.vertcat(
            vx*ca.cos(yaw) - vy*ca.sin(yaw),
            vx*ca.sin(yaw) + vy*ca.cos(yaw),
            omega,
            Fx/m, Fy/m, Mz/I,
        )
        self._f6_fn = ca.Function("f6dof", [x6, T_s, d_s, p], [f6])

    def _setup_solver(self):
        cfg_path = os.path.join(_THIS_DIR, "..", "..", "..", "config", "solver_mpcc_clean.yaml")
        with open(cfg_path) as f:
            self._cfg = yaml.safe_load(f)

        self.N  = self._cfg["solver_creation"]["N"]
        self._Ts = self._cfg["solver_creation"]["Ts"]

        hash_file = os.path.join(_WORKDIR, "data", "mpcc_clean_build_hash.md5")
        os.makedirs(os.path.dirname(hash_file), exist_ok=True)
        needs_rebuild = check_file_changes(
            [
                os.path.join(_THIS_DIR, "pacejka_model.py"),
                os.path.join(_THIS_DIR, "generate_solver.py"),
                cfg_path,
            ],
            hash_file,
        )
        rospy.loginfo(f"[MPCC_PYTHON] rebuild={needs_rebuild}")

        old_cwd = os.getcwd()
        os.chdir(os.path.join(_THIS_DIR, ".."))  # required by acados code generation
        try:
            self._solver = build_solver(self._cfg, _THIS_DIR, rebuild=needs_rebuild)
        finally:
            os.chdir(old_cwd)

    # ── Track helpers ──────────────────────────────────────────────────────────

    def _track_ref(self, theta_i: float) -> dict:
        """Return track reference point at arc-length theta_i."""
        dist = theta_i - self._laps * self._track_max_arc
        idx  = int(dist * self._track_density) % self._track_N
        idx_phi = int(dist * self._track_density) % (2 * self._track_N)
        return {
            "x":      self._track_x[idx],
            "y":      self._track_y[idx],
            "grad_x": self._track_xrate[idx],
            "grad_y": self._track_yrate[idx],
            "theta":  dist + self._laps * self._track_max_arc,
            "phi":    self._track_phi[idx_phi] + self._laps * 2 * math.pi,
        }

    def _build_params(self, ref: dict) -> np.ndarray:
        """Assemble the NP=31 parameter vector for one stage."""
        return np.array([
            ref["x"], ref["y"], ref["grad_x"], ref["grad_y"],
            ref["theta"], ref["phi"],
        ] + self._cost_params + self._model_params)

    def _gen_init_state(self, arc: float, vel: float) -> np.ndarray:
        """Create a 9D state from the track centerline at the given arc length."""
        dist = arc - self._laps * self._track_max_arc
        idx  = int(dist * self._track_density) % self._track_N
        idx_phi = idx % (2 * self._track_N)
        x, y = self._track_x[idx], self._track_y[idx]
        phi  = self._track_phi[idx_phi]
        # Approximate dyaw from curvature
        idx2  = (idx + 1) % self._track_N
        ds    = math.hypot(self._track_x[idx2] - x, self._track_y[idx2] - y) + 1e-9
        kappa = (self._track_xrate[idx] * (self._track_y[idx2] - y)
                 - self._track_yrate[idx] * (self._track_x[idx2] - x)) / ds
        return np.array([x, y, phi, vel, 0.0, kappa * vel, 0.0, 0.0, arc])

    # ── Lag compensation ───────────────────────────────────────────────────────

    def _rk4(self, x6: np.ndarray, T: float, delta: float, dt: float) -> np.ndarray:
        p  = self._dyn_params
        fn = self._f6_fn
        k1 = np.array(fn(x6,            T, delta, p)).flatten()
        k2 = np.array(fn(x6 + dt/2*k1, T, delta, p)).flatten()
        k3 = np.array(fn(x6 + dt/2*k2, T, delta, p)).flatten()
        k4 = np.array(fn(x6 + dt*k3,   T, delta, p)).flatten()
        return x6 + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

    def _lag_compensate(self, x9: np.ndarray) -> np.ndarray:
        """
        Forward-simulate 6-DOF dynamics for lag_compensation_time with the last
        applied torque/steer (same as C++ model_->applyModel).
        Theta is advanced by lag_dt * last_u[0][DTHETA].
        """
        x6_new = self._rk4(x9[:6], self._last_torque, self._last_steer, self._lag_dt)
        x9_out = x9.copy()
        x9_out[:6]          = x6_new
        x9_out[self.IT]     = self._last_torque
        x9_out[self.IDELTA] = self._last_steer
        x9_out[self.ITHETA] = x9[self.ITHETA] + self._lag_dt * self._last_u[0, self.IDTHETA]
        return x9_out

    # ── Solver step ────────────────────────────────────────────────────────────

    def _solve_step(self, x0: np.ndarray):
        """One SQP-RTI step: set x0, warm-start, update params, solve, extract solution."""
        self._solver.set(0, "lbx", x0)
        self._solver.set(0, "ubx", x0)

        for i in range(self.N):
            ref = self._track_ref(self._last_x[i, self.ITHETA])
            self._solver.set(i, "p", self._build_params(ref))
            self._solver.set(i, "x", self._last_x[i])
            self._solver.set(i, "u", self._last_u[i])
            self._last_ref[i] = [ref["x"], ref["y"], ref["phi"]]

        ref_N = self._track_ref(self._last_x[self.N, self.ITHETA])
        self._solver.set(self.N, "p", self._build_params(ref_N))
        self._solver.set(self.N, "x", self._last_x[self.N])

        status = self._solver.solve()
        if status == 0:
            for i in range(self.N + 1):
                self._last_x[i] = self._solver.get(i, "x")
            for i in range(self.N):
                self._last_u[i] = self._solver.get(i, "u")
        else:
            rospy.logwarn_throttle(0.5, f"[MPCC_PYTHON] solver status={status}")

        # Full planned T trajectory — find stage where T first drops to near-zero
        T_plan = [self._last_x[i, self.IT] for i in range(self.N + 1)]
        T_zero_stage = next((i for i, t in enumerate(T_plan) if t < 0.05), None)
        dtheta_plan = [self._last_u[i, self.IDTHETA] for i in range(min(6, self.N))]
        vx_plan     = [self._last_x[i, self.IVX]    for i in range(min(6, self.N + 1))]
        rospy.loginfo_throttle(1.0,
            f"[MPCC_PYTHON] status={status} "
            f"vx={vx_plan[0]:.2f} theta={self._last_x[1,self.ITHETA]:.2f} "
            f"T_zero_at_stage={T_zero_stage} "
            f"T_plan={[f'{t:.2f}' for t in T_plan[:8]]} "
            f"dtheta={[f'{d:.2f}' for d in dtheta_plan]} "
            f"steer={self._last_x[1,self.IDELTA]:.3f}"
        )

    # ── Warmstart ──────────────────────────────────────────────────────────────

    def _initialize(self, state: car_state_cart):
        """
        First-call warmstart matching C++ PacejkaMpccController::initialize().
        Builds initial guess along horizon then runs warmstart_iters solver iterations.
        """
        idx = get_closest_track_point_idx(
            state.x, state.y, self._track_x, self._track_y
        )
        self._theta = float(self._track_arc[idx])
        self._last_torque = 0.5
        self._last_steer  = 0.0

        vx0 = max(0.01, state.vx_b)
        x0 = np.array([
            state.x, state.y, state.yaw,
            vx0, state.vy_b, state.dyaw,
            self._last_torque, self._last_steer, self._theta,
        ])

        # Build initial guess: ramp velocity from vx0 to 1.5 m/s along horizon.
        # T=0.0 in warm-start states (matching C++ generateInitialization); applied
        # torque (self._last_torque=0.5) is pinned at stage 0 via lbx/ubx in _solve_step.
        driven = self._theta
        for i in range(self.N + 1):
            vel = (1.5 - vx0) * i / (self.N + 1) + vx0
            driven += self._Ts * vel
            self._last_x[i] = self._gen_init_state(driven, vel)
            self._last_x[i, self.IT] = 0.0

        rospy.loginfo(f"[MPCC_PYTHON] warmstarting ({self._warmstart_iters} iter) …")
        for _ in range(self._warmstart_iters):
            self._solve_step(x0)
        rospy.loginfo("[MPCC_PYTHON] warmstart done")

        self._initialized = True

    # ── Public interface ───────────────────────────────────────────────────────

    def get_input(self, state: car_state_cart) -> car_input:
        if not self._initialized:
            self._initialize(state)
            msg = car_input()
            msg.torque = 0.5
            msg.steer  = 0.0
            return msg

        # Build 9D state from ROS message + internal theta/last actuation
        x_curr = np.array([
            state.x, state.y, state.yaw,
            state.vx_b, state.vy_b, state.dyaw,
            self._last_torque, self._last_steer, self._theta,
        ])

        # Lag compensation (forward-simulate for lag_compensation_time)
        x_curr = self._lag_compensate(x_curr)
        self._theta = float(x_curr[self.ITHETA])

        # Lap counting
        if self._theta > (self._laps + 1) * self._track_max_arc:
            self._laps += 1

        # Shift warm-start by one step
        self._last_x[:-1] = self._last_x[1:]
        self._last_u[:-1] = self._last_u[1:]

        # SQP iterations
        for _ in range(self._max_sqp_iter):
            self._solve_step(x_curr)

        # Extract output (from stage 1, same as C++)
        self._last_torque = float(self._last_x[1, self.IT])
        self._last_steer  = float(self._last_x[1, self.IDELTA])
        self._theta       = float(self._last_x[1, self.ITHETA])

        # Theta drift diagnostic: compare solver theta with closest physical arc position.
        # If theta_drift >> 0, the solver's virtual progress is ahead of the car's real position
        # and reference track points are ahead of where the car actually is.
        closest_idx  = get_closest_track_point_idx(state.x, state.y, self._track_x, self._track_y)
        theta_actual = float(self._track_arc[closest_idx]) + self._laps * self._track_max_arc
        theta_drift  = self._theta - theta_actual
        rospy.loginfo_throttle(1.0,
            f"[MPCC_PYTHON] theta_solver={self._theta:.3f} theta_arc={theta_actual:.3f} "
            f"drift={theta_drift:+.3f}m  T={self._last_torque:.3f} steer={self._last_steer:.3f}"
        )

        msg = car_input()
        msg.torque = self._last_torque
        msg.steer  = self._last_steer
        return msg

    def publish_visualization(self):
        """Publish planned and reference trajectory markers to RViz."""
        try:
            frame_id = self._vis_params.get("frame_id", "crs_frame")
            ns       = self._vis_params.get("namespace", "mpc")
        except Exception:
            return

        now = rospy.Time.now()

        planned   = Marker()
        planned.header.frame_id = frame_id
        planned.header.stamp    = now
        planned.ns     = ns + "_planned"
        planned.type   = Marker.POINTS
        planned.action = Marker.ADD
        planned.scale.x = 0.04
        planned.scale.y = 0.04
        planned.color.b = 1.0
        planned.color.a = 1.0

        reference   = Marker()
        reference.header.frame_id = frame_id
        reference.header.stamp    = now
        reference.ns     = ns + "_reference"
        reference.type   = Marker.POINTS
        reference.action = Marker.ADD
        reference.scale.x = 0.04
        reference.scale.y = 0.04
        reference.color.r = 1.0
        reference.color.a = 1.0

        from geometry_msgs.msg import Point
        for i in range(self.N):
            p = Point()
            p.x = float(self._last_x[i, self.IX])
            p.y = float(self._last_x[i, self.IY])
            p.z = 0.0
            planned.points.append(p)

            r = Point()
            r.x = float(self._last_ref[i, 0])
            r.y = float(self._last_ref[i, 1])
            r.z = 0.0
            reference.points.append(r)

        self._vis_pub.publish(planned)
        self._vis_pub.publish(reference)

        # Sphere at track point for current theta (where controller thinks car is)
        from std_msgs.msg import ColorRGBA
        theta_now = self._track_ref(self._theta)
        theta_end = self._track_ref(float(self._last_x[self.N, self.ITHETA]))

        for label, ref, color in [
            ("theta_now",  theta_now, ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)),  # green = current theta
            ("theta_end",  theta_end, ColorRGBA(r=1.0, g=0.5, b=0.0, a=1.0)),  # orange = horizon-end theta
        ]:
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp    = now
            m.ns     = ns + "_" + label
            m.id     = 0
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = ref["x"]
            m.pose.position.y = ref["y"]
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.08
            m.scale.y = 0.08
            m.scale.z = 0.08
            m.color  = color
            self._vis_pub.publish(m)

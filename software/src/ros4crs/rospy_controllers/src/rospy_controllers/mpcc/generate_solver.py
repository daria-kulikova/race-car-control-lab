import os
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from .pacejka_model import pacejka_model


def create_ocp(config: dict) -> AcadosOcp:
    """
    Build AcadosOcp that mirrors the C++ pacejka_mpcc solver setup.

    Cost:       EXTERNAL  (same as C++ version)
    Constraint: soft radius² track constraint
    Slacks:     linear penalty on all state/input bounds + track constraint
    """
    bounds = config["model_bounds"]
    N  = config["solver_creation"]["N"]
    Ts = config["solver_creation"]["Ts"]
    slack_lin  = config["solver_creation"].get("slack_penalty_linear", 100.0)
    slack_quad = config["solver_creation"].get("slack_penalty_quadratic", 0.0)

    ocp = AcadosOcp()
    model, constraint = pacejka_model(bounds)

    # ── Acados model ──────────────────────────────────────────────────────────
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x    = model.x
    model_ac.xdot = model.xdot
    model_ac.u    = model.u
    model_ac.z    = model.z
    model_ac.p    = model.p
    model_ac.name = model.name
    model_ac.con_h_expr = constraint.expr
    ocp.model = model_ac

    nx = model.x.size()[0]   # 9
    nu = model.u.size()[0]   # 3
    np_ = model.p.size()[0]  # 31

    # ── Dimensions ────────────────────────────────────────────────────────────
    ocp.dims.nx   = nx
    ocp.dims.nu   = nu
    ocp.dims.np   = np_
    ocp.dims.ny   = 0
    ocp.dims.ny_e = 0
    ocp.dims.nbx  = nx
    ocp.dims.nbu  = nu
    ocp.dims.N    = N

    # soft track constraint
    ocp.dims.nh  = 1
    ocp.dims.nsh = 1

    # soft state/input bounds (intermediate stages)
    ocp.dims.nsbx = nx
    ocp.dims.nsbu = nu
    ocp.dims.ns   = nx + nu + 1  # nsbx + nsbu + nsh

    # ── Cost: EXTERNAL ────────────────────────────────────────────────────────
    ocp.cost.cost_type   = "EXTERNAL"
    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost   = model.cost_expr_ext_cost
    ocp.model.cost_expr_ext_cost_e = 0  # no terminal cost

    # ── Slack penalties (intermediate stages) ─────────────────────────────────
    ns = nx + nu + 1
    ocp.cost.zl = slack_lin  * np.ones(ns)
    ocp.cost.zu = slack_lin  * np.ones(ns)
    ocp.cost.Zl = slack_quad * np.ones(ns)
    ocp.cost.Zu = slack_quad * np.ones(ns)

    # ── State bounds ──────────────────────────────────────────────────────────
    ocp.constraints.lbx = np.array([
        model.x_min, model.y_min, model.yaw_min,
        model.vx_min, model.vy_min, model.dyaw_min,
        model.T_min, model.delta_min, model.theta_min,
    ])
    ocp.constraints.ubx = np.array([
        model.x_max, model.y_max, model.yaw_max,
        model.vx_max, model.vy_max, model.dyaw_max,
        model.T_max, model.delta_max, model.theta_max,
    ])
    ocp.constraints.idxbx  = np.arange(nx)
    ocp.constraints.lsbx   = np.zeros(nx)
    ocp.constraints.usbx   = np.zeros(nx)
    ocp.constraints.idxsbx = np.arange(nx)

    # ── Input bounds ──────────────────────────────────────────────────────────
    ocp.constraints.lbu = np.array([model.dT_min, model.ddelta_min, model.dtheta_min])
    ocp.constraints.ubu = np.array([model.dT_max, model.ddelta_max, model.dtheta_max])
    ocp.constraints.idxbu  = np.arange(nu)
    ocp.constraints.lsbu   = np.zeros(nu)
    ocp.constraints.usbu   = np.zeros(nu)
    ocp.constraints.idxsbu = np.arange(nu)

    # ── Track constraint bound ────────────────────────────────────────────────
    car_width     = bounds.get("car_width", 0.12)
    track_width   = config["track"]["track_width"]
    safety_margin = config["track"]["safety_margin"]
    w = 0.5 * (track_width - car_width) - safety_margin
    lh, uh = constraint.bound_generator(w)
    ocp.constraints.lh    = np.array([lh])
    ocp.constraints.uh    = np.array([uh])
    ocp.constraints.lsh   = np.zeros(1)
    ocp.constraints.ush   = np.zeros(1)
    ocp.constraints.idxsh = np.array([0])

    # ── Initial condition (hard equality, soft only on inputs) ────────────────
    ocp.constraints.x0       = np.zeros(nx)
    ocp.constraints.idxbx_0  = np.arange(nx)
    ocp.cost.zl_0 = slack_lin  * np.ones(nu)
    ocp.cost.zu_0 = slack_lin  * np.ones(nu)
    ocp.cost.Zl_0 = slack_quad * np.ones(nu)
    ocp.cost.Zu_0 = slack_quad * np.ones(nu)

    ocp.parameter_values = np.zeros(np_)

    # ── Solver options ────────────────────────────────────────────────────────
    ocp.solver_options.Tsim                  = Ts
    ocp.solver_options.tf                    = Ts * N
    ocp.solver_options.qp_solver             = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type       = "SQP_RTI"
    ocp.solver_options.hessian_approx        = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type       = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 3
    ocp.solver_options.print_level           = 0
    ocp.solver_options.tol                   = 1e-4

    acados_source = os.environ["ACADOS_SOURCE_DIR"]
    ocp.acados_include_path = acados_source + "/include"
    ocp.acados_lib_path     = acados_source + "/lib"

    return ocp


def build_solver(config: dict, output_dir: str, rebuild: bool = True) -> AcadosOcpSolver:
    """Create (and optionally rebuild) the acados OCP solver."""
    os.makedirs(output_dir, exist_ok=True)
    ocp = create_ocp(config)
    ocp.code_export_directory = os.path.join(output_dir, "c_generated_code")
    json_file = os.path.join(output_dir, "acados_mpcc_ocp.json")
    return AcadosOcpSolver(ocp, json_file=json_file, build=rebuild)

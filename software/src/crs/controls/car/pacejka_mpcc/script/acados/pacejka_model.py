import math
import os
import numpy as np
from casadi import *
from typing import Dict, Optional


def pacejka_model(
    constraints: Dict[str, float],
    use_linear_track_constraint: bool = False,
    mlp_weights_path: Optional[str] = None,
    Ts: float = 0.02,
    residual_scale: float = 1.0,
):
    # define structs
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model_name = "pacejka_model"

    """ constraints """
    model.x_min = constraints["x_min"]
    model.x_max = constraints["x_max"]
    model.y_min = constraints["y_min"]
    model.y_max = constraints["y_max"]
    model.yaw_min = constraints["yaw_min"]
    model.yaw_max = constraints["yaw_max"]
    model.vx_min = constraints["vx_min"]
    model.vx_max = constraints["vx_max"]
    model.vy_min = constraints["vy_min"]
    model.vy_max = constraints["vy_max"]
    model.dyaw_min = constraints["dyaw_min"]
    model.dyaw_max = constraints["dyaw_max"]
    model.delta_min = constraints["delta_min"]
    model.delta_max = constraints["delta_max"]
    model.T_min = constraints["T_min"]
    model.T_max = constraints["T_max"]
    model.theta_min = constraints["theta_min"]
    model.theta_max = constraints["theta_max"]
    model.ddelta_min = constraints["ddelta_min"]
    model.ddelta_max = constraints["ddelta_max"]
    model.dT_min = constraints["dT_min"]
    model.dT_max = constraints["dT_max"]
    model.dtheta_min = constraints["dtheta_min"]
    model.dtheta_max = constraints["dtheta_max"]

    """ Dynamics """
    # CasADi - states
    xp = SX.sym("xp")
    yp = SX.sym("yp")
    yaw = SX.sym("yaw")
    vx = SX.sym("vx")
    vy = SX.sym("vy")
    omega = SX.sym("omega")
    T = SX.sym("T")
    delta = SX.sym("delta")
    theta = SX.sym("theta")
    x = vertcat(xp, yp, yaw, vx, vy, omega, T, delta, theta)

    # CasADi - input
    dT = SX.sym("dT")
    ddelta = SX.sym("ddelta")
    dtheta = SX.sym("dtheta")
    u = vertcat(dT, ddelta, dtheta)

    # xdot
    xpdot = SX.sym("xpdot")
    ypdot = SX.sym("ypdot")
    yawdot = SX.sym("yawdot")
    vxdot = SX.sym("vxdot")
    vydot = SX.sym("vydot")
    omegadot = SX.sym("omegadot")
    xdot = vertcat(xpdot, ypdot, yawdot, vxdot, vydot, omegadot, dT, ddelta, dtheta)

    # algebraic variables
    z = vertcat([])

    # define params
    xd = SX.sym("xd")
    yd = SX.sym("yd")
    grad_xd = SX.sym("grad_xd")
    grad_yd = SX.sym("grad_yd")
    theta_hat = SX.sym("theta_hat")
    phi_d = SX.sym("phi_d")
    Q1 = SX.sym("Q1")
    Q2 = SX.sym("Q2")
    R1 = SX.sym("R1")
    R2 = SX.sym("R2")
    R3 = SX.sym("R3")
    q = SX.sym("q")
    # Model params
    lr = SX.sym("lr")
    lf = SX.sym("lf")
    m = SX.sym("m")
    I = SX.sym("I")
    Df = SX.sym("Df")
    Cf = SX.sym("Cf")
    Bf = SX.sym("Bf")
    Dr = SX.sym("Dr")
    Cr = SX.sym("Cr")
    Br = SX.sym("Br")
    Cm1 = SX.sym("Cm1")
    Cm2 = SX.sym("Cm2")
    Cd0 = SX.sym("Cd0")
    Cd1 = SX.sym("Cd1")
    Cd2 = SX.sym("Cd2")
    gamma = SX.sym("gamma")
    eps = SX.sym("eps")
    car_width = SX.sym("car_width")
    wheel_radius = SX.sym("wheel_radius")

    # GP residual parameters (zero-order correction, constant over horizon)
    d_vx = SX.sym("d_vx")
    d_vy = SX.sym("d_vy")
    d_omega = SX.sym("d_omega")

    # parameters
    p = vertcat(
        xd,
        yd,
        grad_xd,
        grad_yd,
        theta_hat,
        phi_d,
        Q1,
        Q2,
        R1,
        R2,
        R3,
        q,
        lr,
        lf,
        m,
        I,
        Df,
        Cf,
        Bf,
        Dr,
        Cr,
        Br,
        Cm1,
        Cm2,
        Cd0,
        Cd1,
        Cd2,
        gamma,
        eps,
        car_width,
        wheel_radius,
        d_vx,
        d_vy,
        d_omega,
    )

    #   Slip Angles
    #   Approximation of arctan(w/x) with a polynomial of degree 3
    #   to overcome the discontinuity at x=0. This is needed to avoid
    #   numerical problems with the atan function. The approximation
    #   is applied for x < eps and has the same value and derivative at
    #   x=eps as the atan function.

    #   The polynomial is given by:
    #   g(x) = b*x + c*x*x*x;
    #   b = 1/2 * w/(eps^2+w^2) + 3/(2*eps)*arctan(w/eps)
    #   c = -1/(2*eps^3)* arctan(w/eps) - w/(2eps^2) * 1/(w*w + eps*eps)

    w_r = -vy + lr * omega
    b_r = 0.5 * w_r / (eps * eps + w_r * w_r) + 3 / (2 * eps) * atan(w_r / eps)
    c_r = -1 / (2 * eps * eps * eps) * atan(w_r / eps) - w_r / (2 * eps * eps) * 1 / (
        w_r * w_r + eps * eps
    )
    g_r = b_r * vx + c_r * vx * vx * vx

    w_f = -vy - lf * omega
    b_f = 0.5 * w_f / (eps * eps + w_f * w_f) + 3 / (2 * eps) * atan(w_f / eps)
    c_f = -1 / (2 * eps * eps * eps) * atan(w_f / eps) - w_f / (2 * eps * eps) * 1 / (
        w_f * w_f + eps * eps
    )
    g_f = b_f * vx + c_f * vx * vx * vx

    ar = if_else(vx < eps, g_r, atan2(-vy + lr * omega, vx))
    af = if_else(vx < eps, g_f, delta + atan2(-vy - lf * omega, vx))

    # forces on the wheels
    Fm = (Cm1 - Cm2 * vx) * T  # motor force
    Ffriction = sign(vx) * (-Cd0 - Cd1 * vx - Cd2 * vx * vx)  # friction force

    Fx_f = Fm * (1 - gamma)  # front wheel force, x component
    Fx_r = Fm * gamma  # rear wheel force, x component

    Fy_f = Df * sin(Cf * atan(Bf * af))  # front wheel force, y component
    Fy_r = Dr * sin(Cr * atan(Br * ar))  # rear wheel force, y component

    # dynamics including apparent forces
    Fx = Fx_r + Fx_f * cos(delta) - Fy_f * sin(delta) + m * vy * omega + Ffriction
    Fy = Fy_r + Fx_f * sin(delta) + Fy_f * cos(delta) - m * vx * omega
    Mz = Fy_f * lf * cos(delta) + Fx_f * lf * sin(delta) - Fy_r * lr

    # Residual correction: MLP (per-stage, build-time) or parametric d_vx/d_vy/d_omega (zero-order, runtime)
    if mlp_weights_path and os.path.exists(mlp_weights_path):
        npz = np.load(mlp_weights_path)
        x_mean_np = npz["x_mean"]; x_std_np = npz["x_std"]
        y_mean_np = npz["y_mean"]; y_std_np = npz["y_std"]
        hidden    = list(npz["hidden"])
        n_layers  = len(hidden) + 1   # hidden layers + output layer

        # Normalize input
        x_in  = vertcat(vx, vy, omega, delta, T)
        act   = (x_in - DM(x_mean_np)) / DM(x_std_np)

        # Forward pass: tanh for hidden layers, linear for output
        for i in range(n_layers):
            W_np = npz[f"W{i}"]
            b_np = npz[f"b{i}"]
            pre  = DM(W_np) @ act + DM(b_np)
            act  = tanh(pre) if i < n_layers - 1 else pre

        # Unnormalize, scale, and convert velocity residual [m/s] → acceleration [m/s²]
        out = act * DM(y_std_np) + DM(y_mean_np)
        d_vx_eff    = residual_scale * out[0] / Ts
        d_vy_eff    = residual_scale * out[1] / Ts
        d_omega_eff = residual_scale * out[2] / Ts
        print(f"[pacejka_model] MLP embedded (arch [5]+{hidden}+[3], scale={residual_scale}): {mlp_weights_path}")
    else:
        # Parametric fallback: d_vx/d_vy/d_omega set at runtime via setGPResidual()
        # Scaling is applied in C++ controller before calling setGPResidual()
        d_vx_eff    = d_vx
        d_vy_eff    = d_vy
        d_omega_eff = d_omega

    f_expl = vertcat(
        vx * cos(yaw) - vy * sin(yaw),
        vx * sin(yaw) + vy * cos(yaw),
        omega,
        Fx / m + d_vx_eff,
        Fy / m + d_vy_eff,
        Mz / I + d_omega_eff,
        dT,
        ddelta,
        dtheta,
    )

    # cost
    eC = sin(phi_d) * (xp - xd - grad_xd * (theta - theta_hat)) - cos(phi_d) * (
        yp - yd - grad_yd * (theta - theta_hat)
    )
    eL = -cos(phi_d) * (xp - xd - grad_xd * (theta - theta_hat)) - sin(phi_d) * (
        yp - yd - grad_yd * (theta - theta_hat)
    )

    c_eC = eC * eC * Q1
    c_eL = eL * eL * Q2
    c_theta = -q * theta
    c_dT = dT * dT * R1
    c_ddelta = ddelta * ddelta * R2
    c_dtheta = dtheta * dtheta * R3

    model.cost_expr_ext_cost = c_eC + c_eL + c_theta + c_dT + c_ddelta + c_dtheta

    # nonlinear track constraints
    xdiff, ydiff = (xp - xd), (yp - yd)
    if use_linear_track_constraint:
        constraint.expr = vertcat(
            -grad_yd * xdiff + grad_xd * ydiff,
        )
        constraint.bound_generator = lambda track_width: (-track_width, track_width)
    else:
        radius_sq = xdiff * xdiff + ydiff * ydiff
        constraint.expr = vertcat(radius_sq)
        constraint.bound_generator = lambda track_width: (0, track_width * track_width)

    params = types.SimpleNamespace()
    params.xd = xd
    params.yd = yd
    params.grad_xd = grad_xd
    params.grad_yd = grad_yd
    params.phi_d = phi_d
    params.theta_hat = theta_hat
    params.Q1 = Q1
    params.Q2 = Q2
    params.R1 = R1
    params.R2 = R2
    params.R3 = R3
    params.q = q
    params.lr = lr
    params.lf = lf
    params.m = m
    params.I = I
    params.Df = Df
    params.Cf = Cf
    params.Bf = Bf
    params.Dr = Dr
    params.Cr = Cr
    params.Br = Br
    params.Cm1 = Cm1
    params.Cm2 = Cm2
    params.Cd0 = Cd0
    params.Cd1 = Cd1
    params.Cd2 = Cd2
    params.gamma = gamma
    params.eps = eps
    params.car_width = car_width
    params.gamma = gamma

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x = x
    model.xdot = xdot
    model.u = u
    model.z = z
    model.p = p
    model.name = model_name
    model.params = params

    # return model, constraint
    return model, constraint

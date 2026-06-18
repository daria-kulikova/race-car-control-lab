from casadi import *
from typing import Dict


def pacejka_model(constraints: Dict[str, float]):
    """
    CasADi model for MPCC with Pacejka tire model.

    State:  x = [xp, yp, yaw, vx, vy, omega, T, delta, theta]  (NX=9)
    Input:  u = [dT, ddelta, dtheta]                             (NU=3)
    Params: p = [xd, yd, grad_xd, grad_yd, theta_hat, phi_d,    (NP=31)
                 Q1, Q2, R1, R2, R3, q,
                 lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br,
                 Cm1, Cm2, Cd0, Cd1, Cd2, gamma, eps,
                 car_width, wheel_radius]

    Mirrors pacejka_mpcc/script/acados/pacejka_model.py from the C++ package.
    """
    model = types.SimpleNamespace()
    constraint = types.SimpleNamespace()
    model.name = "pacejka_mpcc_py"

    # ── Bounds ────────────────────────────────────────────────────────────────
    for k, v in constraints.items():
        setattr(model, k, v)

    # ── Symbolic state ────────────────────────────────────────────────────────
    xp    = SX.sym("xp")
    yp    = SX.sym("yp")
    yaw   = SX.sym("yaw")
    vx    = SX.sym("vx")
    vy    = SX.sym("vy")
    omega = SX.sym("omega")
    T     = SX.sym("T")
    delta = SX.sym("delta")
    theta = SX.sym("theta")
    x = vertcat(xp, yp, yaw, vx, vy, omega, T, delta, theta)

    # ── Symbolic input ────────────────────────────────────────────────────────
    dT     = SX.sym("dT")
    ddelta = SX.sym("ddelta")
    dtheta = SX.sym("dtheta")
    u = vertcat(dT, ddelta, dtheta)

    # ── xdot (implicit form) ──────────────────────────────────────────────────
    xpdot    = SX.sym("xpdot")
    ypdot    = SX.sym("ypdot")
    yawdot   = SX.sym("yawdot")
    vxdot    = SX.sym("vxdot")
    vydot    = SX.sym("vydot")
    omegadot = SX.sym("omegadot")
    xdot = vertcat(xpdot, ypdot, yawdot, vxdot, vydot, omegadot, dT, ddelta, dtheta)

    z = vertcat([])  # no algebraic variables

    # ── Symbolic parameters ───────────────────────────────────────────────────
    xd       = SX.sym("xd")
    yd       = SX.sym("yd")
    grad_xd  = SX.sym("grad_xd")
    grad_yd  = SX.sym("grad_yd")
    theta_hat = SX.sym("theta_hat")
    phi_d    = SX.sym("phi_d")
    Q1  = SX.sym("Q1")
    Q2  = SX.sym("Q2")
    R1  = SX.sym("R1")
    R2  = SX.sym("R2")
    R3  = SX.sym("R3")
    q   = SX.sym("q")
    lr  = SX.sym("lr")
    lf  = SX.sym("lf")
    m   = SX.sym("m")
    I   = SX.sym("I")
    Df  = SX.sym("Df")
    Cf  = SX.sym("Cf")
    Bf  = SX.sym("Bf")
    Dr  = SX.sym("Dr")
    Cr  = SX.sym("Cr")
    Br  = SX.sym("Br")
    Cm1 = SX.sym("Cm1")
    Cm2 = SX.sym("Cm2")
    Cd0 = SX.sym("Cd0")
    Cd1 = SX.sym("Cd1")
    Cd2 = SX.sym("Cd2")
    gamma      = SX.sym("gamma")
    eps        = SX.sym("eps")
    car_width  = SX.sym("car_width")
    wheel_radius = SX.sym("wheel_radius")

    p = vertcat(
        xd, yd, grad_xd, grad_yd, theta_hat, phi_d,
        Q1, Q2, R1, R2, R3, q,
        lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br,
        Cm1, Cm2, Cd0, Cd1, Cd2,
        gamma, eps, car_width, wheel_radius,
    )

    # ── Pacejka slip angles (polynomial approximation near vx=0) ─────────────
    w_r = -vy + lr * omega
    b_r = 0.5 * w_r / (eps*eps + w_r*w_r) + 3/(2*eps) * atan(w_r / eps)
    c_r = -1/(2*eps**3) * atan(w_r / eps) - w_r/(2*eps**2) / (w_r*w_r + eps*eps)
    g_r = b_r * vx + c_r * vx**3

    w_f = -vy - lf * omega
    b_f = 0.5 * w_f / (eps*eps + w_f*w_f) + 3/(2*eps) * atan(w_f / eps)
    c_f = -1/(2*eps**3) * atan(w_f / eps) - w_f/(2*eps**2) / (w_f*w_f + eps*eps)
    g_f = b_f * vx + c_f * vx**3

    ar = if_else(vx < eps, g_r, atan2(-vy + lr * omega, vx))
    af = if_else(vx < eps, g_f, delta + atan2(-vy - lf * omega, vx))

    # ── Forces ────────────────────────────────────────────────────────────────
    Fm        = (Cm1 - Cm2 * vx) * T
    Ffriction = sign(vx) * (-Cd0 - Cd1 * vx - Cd2 * vx**2)
    Fx_f = Fm * (1 - gamma)
    Fx_r = Fm * gamma
    Fy_f = Df * sin(Cf * atan(Bf * af))
    Fy_r = Dr * sin(Cr * atan(Br * ar))

    Fx = Fx_r + Fx_f*cos(delta) - Fy_f*sin(delta) + m*vy*omega + Ffriction
    Fy = Fy_r + Fx_f*sin(delta) + Fy_f*cos(delta) - m*vx*omega
    Mz = Fy_f*lf*cos(delta) + Fx_f*lf*sin(delta) - Fy_r*lr

    # ── Explicit dynamics ─────────────────────────────────────────────────────
    f_expl = vertcat(
        vx*cos(yaw) - vy*sin(yaw),
        vx*sin(yaw) + vy*cos(yaw),
        omega,
        Fx / m,
        Fy / m,
        Mz / I,
        dT,
        ddelta,
        dtheta,
    )

    # ── MPCC cost (EXTERNAL) ──────────────────────────────────────────────────
    eC = ( sin(phi_d) * (xp - xd - grad_xd*(theta - theta_hat))
          - cos(phi_d) * (yp - yd - grad_yd*(theta - theta_hat)))
    eL = (-cos(phi_d) * (xp - xd - grad_xd*(theta - theta_hat))
          - sin(phi_d) * (yp - yd - grad_yd*(theta - theta_hat)))

    model.cost_expr_ext_cost = (
        Q1 * eC**2
        + Q2 * eL**2
        - q * theta
        + R1 * dT**2
        + R2 * ddelta**2
        + R3 * dtheta**2
    )

    # ── Track constraint: radius² ≤ w² ───────────────────────────────────────
    xdiff = xp - xd
    ydiff = yp - yd
    constraint.expr = vertcat(xdiff**2 + ydiff**2)
    constraint.bound_generator = lambda w: (0.0, w * w)

    model.f_impl_expr = xdot - f_expl
    model.f_expl_expr = f_expl
    model.x    = x
    model.xdot = xdot
    model.u    = u
    model.z    = z
    model.p    = p

    return model, constraint

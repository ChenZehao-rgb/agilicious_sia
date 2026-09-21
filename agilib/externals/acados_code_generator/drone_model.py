"""Generate the ideal-inner-loop collective-thrust/body-rate MPC.

State: [position_ENU(3), quaternion_wxyz_body_to_world(4), velocity_ENU(3)].
Input: [collective_acceleration_m_s2, body_rate_FLU_rad_s(3)].
Only the reference quaternion is an online model parameter. The downstream
flight controller is assumed to track body-rate and collective commands
instantaneously; no measured motor or closed-loop delay is claimed here.
"""

import argparse

import casadi as ca
import future_fstrings
import numpy as np
import scipy.linalg

# The pinned acados checkout uses this encoding; explicit registration also
# supports dependencies installed with pip --target and exposed via PYTHONPATH.
future_fstrings.register()
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

GRAVITY = 9.8066
HORIZON_STEPS = 20
HORIZON_SECONDS = 1.0


def quat_mult(q1, q2):
    return ca.vertcat(
        q1[0] * q2[0] - ca.dot(q1[1:4], q2[1:4]),
        q1[0] * q2[1:4] + q2[0] * q1[1:4] + ca.cross(q1[1:4], q2[1:4]),
    )


def quat_error(q, q_ref):
    error = quat_mult(ca.vertcat(q[0], -q[1:4]), q_ref)
    # Preserve the existing tilt-prioritized nonlinear least-squares residual.
    denominator = ca.sqrt(error[0] ** 2 + error[3] ** 2 + 1e-3)
    return ca.vertcat(
        error[0] * error[1] - error[2] * error[3],
        error[0] * error[2] + error[1] * error[3],
        error[3],
    ) / denominator


def rotate_quat(q, vector):
    return quat_mult(
        quat_mult(q, ca.vertcat(0, vector)), ca.vertcat(q[0], -q[1:4])
    )[1:4]


def drone_model():
    model = AcadosModel()
    model.name = "drone_model"
    position = ca.SX.sym("position", 3)
    attitude = ca.SX.sym("attitude", 4)
    velocity = ca.SX.sym("velocity", 3)
    collective = ca.SX.sym("collective_acceleration")
    bodyrates = ca.SX.sym("bodyrates", 3)
    reference_attitude = ca.SX.sym("reference_attitude", 4)

    model.x = ca.vertcat(position, attitude, velocity)
    model.xdot = ca.SX.sym("state_derivative", 10)
    model.u = ca.vertcat(collective, bodyrates)
    model.z = ca.SX.sym("algebraic", 0)
    model.p = reference_attitude
    model.f_expl_expr = ca.vertcat(
        velocity,
        0.5 * quat_mult(attitude, ca.vertcat(0, bodyrates)),
        rotate_quat(attitude, ca.vertcat(0, 0, collective))
        + ca.DM([0.0, 0.0, -GRAVITY]),
    )
    model.f_impl_expr = model.xdot - model.f_expl_expr
    state_residual = ca.vertcat(position, quat_error(attitude, reference_attitude), velocity)
    model.cost_y_expr = ca.vertcat(state_residual, model.u)
    model.cost_y_expr_e = state_residual
    return model


def acados_settings(N=HORIZON_STEPS, Tf=HORIZON_SECONDS):
    ocp = AcadosOcp()
    ocp.model = drone_model()
    ocp.dims.N = N
    ocp.parameter_values = np.array([1.0, 0.0, 0.0, 0.0])

    # Generation defaults only. Runtime configuration replaces all four bounds.
    # The first bound is T/m in m/s^2; the remaining three are body rates in rad/s.
    ocp.constraints.idxbu = np.arange(4, dtype=np.int64)
    ocp.constraints.lbu = np.array([0.0, -10.0, -10.0, -4.0])
    ocp.constraints.ubu = np.array([30.0, 10.0, 10.0, 4.0])
    ocp.constraints.x0 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    state_weights = np.diag([100.0, 100.0, 100.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    input_weights = np.diag([1.0, 1.0, 1.0, 1.0])
    ocp.cost.W = scipy.linalg.block_diag(state_weights, input_weights)
    ocp.cost.W_e = state_weights
    ocp.cost.yref = np.concatenate([np.zeros(9), [GRAVITY, 0.0, 0.0, 0.0]])
    ocp.cost.yref_e = np.zeros(9)

    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    return AcadosOcpSolver(ocp, json_file="acados_ocp.json"), ocp.model


def check_model(model):
    """Check physical units, quaternion kinematics and cost dimensions before copying C."""
    assert (model.x.size1(), model.u.size1(), model.p.size1()) == (10, 4, 4)
    assert (model.cost_y_expr.size1(), model.cost_y_expr_e.size1()) == (13, 9)
    dynamics = ca.Function("check_dynamics", [model.x, model.u], [model.f_expl_expr])
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    hover = np.array([GRAVITY, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(np.array(dynamics(state, hover)).ravel(), np.zeros(10), atol=1e-12)
    rotating = hover.copy()
    rotating[3] = 2.0
    expected = np.zeros(10)
    expected[6] = 1.0
    np.testing.assert_allclose(np.array(dynamics(state, rotating)).ravel(), expected, atol=1e-12)
    pitch = 0.2
    state[3:7] = [np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0]
    expected[6] = 0.0
    expected[7:10] = [GRAVITY * np.sin(pitch), 0.0, GRAVITY * (np.cos(pitch) - 1.0)]
    np.testing.assert_allclose(np.array(dynamics(state, hover)).ravel(), expected, atol=1e-12)
    print("Model checks passed: NX=10 NU=4 NP=4 NY=13 NYN=9; hover, yaw rate, tilted thrust.")


def check_solver(solver):
    state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    hover = np.array([GRAVITY, 0.0, 0.0, 0.0])
    for stage in range(HORIZON_STEPS + 1):
        solver.set(stage, "x", state)
        if stage < HORIZON_STEPS:
            solver.set(stage, "u", hover)
    for _ in range(5):
        assert solver.solve() == 0, "Generated solver failed hover validation"
    np.testing.assert_allclose(solver.get(0, "u"), hover, atol=1e-7)
    np.testing.assert_allclose(solver.get(HORIZON_STEPS, "x"), state, atol=1e-7)
    print("Generated solver hover validation passed.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-model-only", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    check_model(drone_model())
    if not args.check_model_only:
        generated_solver, _ = acados_settings()
        if args.self_test:
            check_solver(generated_solver)

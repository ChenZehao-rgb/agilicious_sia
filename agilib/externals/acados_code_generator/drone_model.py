import casadi as ca
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
import scipy.linalg
import numpy as np

def quat_mult(q1,q2):
    ans = ca.vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
                  q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
                  q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
                  q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

def quat_error(q, q_ref):
    q_aux = ca.vertcat(q[0,:] * q_ref[0,:] + q[1,:] * q_ref[1,:] + q[2,:] * q_ref[2,:] + q[3,:] * q_ref[3,:],
                  -q[1,:] * q_ref[0,:] + q[0,:] * q_ref[1,:] + q[3,:] * q_ref[2,:] - q[2,:] * q_ref[3,:],
                  -q[2,:] * q_ref[0,:] - q[3,:] * q_ref[1,:] + q[0,:] * q_ref[2,:] + q[1,:] * q_ref[3,:],
                  -q[3,:] * q_ref[0,:] + q[2,:] * q_ref[1,:] - q[1,:] * q_ref[2,:] + q[0,:] * q_ref[3,:])
    # attitude errors. SQRT have small quantities added (1e-3) to alleviate the derivative
    # not being defined at zero, and also because it's in the denominator
    q_att_denom = ca.sqrt(q_aux[0] * q_aux[0] + q_aux[3] * q_aux[3] + 1e-3)
    q_att = ca.vertcat(
        q_aux[0] * q_aux[1] - q_aux[2] * q_aux[3],
        q_aux[0] * q_aux[2] + q_aux[1] * q_aux[3],
        q_aux[3]) / q_att_denom
    return q_att

def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return ca.vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec


def drone_model():
    # define structs
    model = ca.types.SimpleNamespace()
    params = ca.types.SimpleNamespace()

    model_name = "drone_model"

    # Drone online parameters
    m = ca.SX.sym('mass')
    q_ref = ca.SX.sym('quat_ref', 4)
    cd = ca.SX.sym('cd', 7)        # cdx1, cdy1, cdz1, cdx3, cdy3, cdz3, cdzh
    l_x = ca.SX.sym('l_x', 4)
    l_y = ca.SX.sym('l_y', 4)
    kappa = ca.SX.sym('kappa')
    inertia_diag = ca.SX.sym('inertia', 3)

    online_params = ca.vertcat(m, q_ref, cd, l_x, l_y, kappa, inertia_diag)

    arm_length = 0.11           # m
    inertia = ca.SX.eye(3)
    inertia[0,0] = inertia_diag[0]
    inertia[1,1] = inertia_diag[1]
    inertia[2,2] = inertia_diag[2]

    inertia_inv = ca.SX.eye(3)
    inertia_inv[0,0] = 1/inertia_diag[0]
    inertia_inv[1,1] = 1/inertia_diag[1]
    inertia_inv[2,2] = 1/inertia_diag[2]
    motor_tau = 0.033                     # s

    omega_max = [10.0, 10.0, 4.0]         # [rad/s]
    thrust_min = 0.0                      # [N]
    thrust_max = 8.5                      # [N] per motor

    # Fill params:
    # params.m = m

    ## CasADi Model
    p = ca.SX.sym('p', 3)
    q = ca.SX.sym('q', 4)                                    # From body to world
    q_conj = ca.vertcat(q[0,:], -q[1,:], -q[2, :], -q[3, :])  # From world to body
    v = ca.SX.sym('v', 3)
    w = ca.SX.sym('w', 3)
    T = ca.SX.sym('thrust', 4)

    # Aerodynamics:
    v_B = rotate_quat(q_conj, v)

    Fd_B = ca.vertcat(cd[0] * v_B[0] + cd[3] * v_B[0]**3,
                   cd[1] * v_B[1] + cd[4] * v_B[1]**3,
                   cd[2] * v_B[2] + cd[5] * v_B[2]**3 - cd[6] * (v_B[0]**2 + v_B[1]**2))
    Fd_W = rotate_quat(q, Fd_B)

    # Dynamics:
    x = ca.vertcat(p, q, v, w)
    u = ca.vertcat(T)

    p_dot = ca.SX.sym('p_dot', 3)
    q_dot = ca.SX.sym('q_dot', 4)
    v_dot = ca.SX.sym('v_dot', 3)
    w_dot = ca.SX.sym('w_dot', 3)
    x_dot = ca.vertcat(p_dot, q_dot, v_dot, w_dot)

    g = ca.DM([0, 0, -9.8066])

    # moments:
    t_BM = ca.horzcat(-l_x, l_y)

    tau_yx = ca.mtimes(t_BM.T, T)

    f_expl = ca.vertcat(
        v,                      # p_dot
        0.5*quat_mult(q, ca.vertcat(0, w)),  # q_dot
        rotate_quat(q, ca.vertcat(0, 0, (T[0]+T[1]+T[2]+T[3])/m)) + g - Fd_W/m,  # v_dot
        ca.mtimes(inertia_inv, ca.vertcat(                                 # w _dot
            tau_yx[1],
            tau_yx[0],
            kappa*(-T[0]-T[1]+T[2]+T[3]))
               -ca.cross(w, ca.mtimes(inertia,w)))
    )

    # algebraic variables
    z = ca.vertcat([])
    q_att = quat_error(q, q_ref)


    # Fill model:
    model.f_expl = f_expl
    model.f_impl = x_dot - f_expl
    model.x = x
    model.x_dot = x_dot
    model.u = u
    model.z = z
    model.cost_y_expr = ca.vertcat(p, q_att, v, w, u)
    model.cost_y_expr_e = ca.vertcat(p, q_att, v, w)
    model.p = online_params
    model.name = model_name

    return model


def acados_settings(N, Tf):
    ocp = AcadosOcp()
    model_drone = drone_model()

    # Acados ODE
    model_acados = AcadosModel()
    model_acados.f_impl_expr = model_drone.f_impl
    model_acados.f_expl_expr = model_drone.f_expl
    model_acados.x = model_drone.x
    model_acados.xdot = model_drone.x_dot
    model_acados.u = model_drone.u
    model_acados.z = model_drone.z
    model_acados.cost_y_expr = model_drone.cost_y_expr
    model_acados.cost_y_expr_e = model_drone.cost_y_expr_e
    model_acados.p = model_drone.p
    model_acados.name = model_drone.name
    ocp.model = model_acados

    mass_value = 0.752          # kg
    q_ref_init = np.array([1.0, 0.0, 0.0, 0.0])
    cd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    l_x = np.array([0.075, -0.075, -0.075, 0.075])
    l_y = np.array([-0.10, 0.10, -0.10, 0.10])
    kappa = 0.022
    inertia_diag = np.array([0.0025, 0.0021, 0.0043])
    ocp.parameter_values = np.concatenate([np.array([mass_value]), q_ref_init, cd, l_x, l_y,
                                           np.array([kappa]), inertia_diag])

    # dimensions
    nx = model_drone.x.size()[0]
    nu = model_drone.u.size()[0]
    ny = nx + nu
    ny_e = nx

    ocp.dims.N = N             # Discretization steps

    # Constraints
    omega_max = [10.0, 10.0, 4.0]         # [rad/s]
    thrust_min = 0.0                      # [N]
    thrust_max = 8.5                      # [N] per motor

    Jbx = np.zeros((3, nx))
    Jbx[0, 10] = 1.0
    Jbx[1, 11] = 1.0
    Jbx[2, 12] = 1.0
    ocp.constraints.Jbx = Jbx
    ocp.constraints.lbx = -10 * np.ones((3,))
    ocp.constraints.ubx = 10 * np.ones((3,))

    Jbu = np.identity(nu)
    ocp.constraints.Jbu = Jbu
    ocp.constraints.lbu = thrust_min * np.ones((nu,))
    ocp.constraints.ubu = thrust_max * np.ones((nu,))


    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"


    Q = np.diag([100, 100, 100, 1, 1, 1, 1, 1, 1, 10, 10, 10])
    R = np.diag([1, 1, 1, 1])
    ocp.cost.W = scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Q

    hover_prop = mass_value * 9.8066 / 4.0

    # initial references
    ocp.cost.yref = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, hover_prop, hover_prop, hover_prop, hover_prop])
    ocp.cost.yref_e = np.array([0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])

    # initial state
    ocp.constraints.x0 = np.array([0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    ocp.solver_options.tf = Tf
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"  # "PARTIAL_CONDENSING_HPIPM", "FULL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"     # "SQP", "SQP_RTI"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"  # "GAUSS_NEWTON", "EXACT"
    ocp.solver_options.integrator_type = "ERK"   # "ERK", "IRK", "GNSF"
    #ocp.solver_options.sim_method_jac_reuse = True
    #ocp.solver_options.sim_method_newton_iter = 1
    #ocp.solver_options.sim_method_num_stages = 3
    #ocp.solver_options.sim_method_num_steps = 1
    acados_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
    return acados_solver, model_drone


N = 20
Tf = 1.0
T = 10.0

Nsim = int(T * N / Tf)

acados_solver, model_drone = acados_settings(N, Tf)

for i in range(Nsim):
    status = acados_solver.solve()
    if status != 0:
        print("acados returned status {} in closed loop iteration {}.".format(status, i))

    # get solution
    x_sol = acados_solver.get(0, "x")
    u_sol = acados_solver.get(0, "u")

    x1 = acados_solver.get(1, "x")
    acados_solver.set(0, "lbx", x1)
    acados_solver.set(0, "ubx", x1)
    print(x1)
    print(u_sol)

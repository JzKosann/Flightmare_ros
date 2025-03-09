from casadi import SX, vertcat, sin, cos, tan
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import numpy as np

def quadrotor_model():

    model_name = 'quadrotor'

    # parameter
    m = 1.0  # 质量
    g = 9.81  # 重力加速度
    Ixx = 0.1  # 转动惯量
    Iyy = 0.1
    Izz = 0.1
    J_RP = 0.01  # 旋翼转动惯量
    d = 0.2 # 旋翼到无人机中心的距离（臂长）
    Cm = 7.3e-06 # 转矩系数
    Ct = 3.25e-4 # 拉力系数

    # 状态变量
    # 世界坐标
    x = SX.sym('x')
    y = SX.sym('y')
    z = SX.sym('z')
    # 世界坐标位置
    vx = SX.sym('vx')
    vy = SX.sym('vy')
    vz = SX.sym('vz')
    # 欧拉角
    phi = SX.sym('phi')     # roll
    theta = SX.sym('theta') # pitch
    psi = SX.sym('psi')     # yaw
    # 角速度
    p = SX.sym('p')     # wx
    q = SX.sym('q')     # wy
    r = SX.sym('r')     # wz
    states = vertcat(x, y, z, vx, vy, vz, phi, theta, psi, p, q, r)

    # 控制输入
    # f = SX.sym('f')
    # tau_x = SX.sym('tau_x')
    # tau_y = SX.sym('tau_y')
    # tau_z = SX.sym('tau_z')
    # controls = vertcat(f, tau_x, tau_y, tau_z)
    w1 = SX.sym('w1')
    w2 = SX.sym('w2')
    w3 = SX.sym('w3')
    w4 = SX.sym('w4')
    controls = vertcat(w1, w2, w3, w4)
    f = Ct * (w1**2 + w2**2 + w3**2 + w4**2)
    Omega = w1 - w2 + w3 - w4
    tau_x = d * Ct * (np.sqrt(2) / 2) * ( w1**2 - w2**2 - w3**2 + w4**2 )
    tau_y = d * Ct * (np.sqrt(2) / 2) * ( - w1**2 - w2**2 + w3**2 + w4**2 )
    tau_z = Cm * ( - w1**2 + w2**2 - w3**2 + w4**2)
    # tau_z = d * Cm * ( - w1**2 + w2**2 - w3**2 + w4**2)

    # 状态变量 求导
    # 世界坐标
    x_dot = SX.sym('x_dot')
    y_dot = SX.sym('y_dot')
    z_dot = SX.sym('z_dot')
    # 世界坐标位置
    vx_dot = SX.sym('vx_dot')
    vy_dot = SX.sym('vy_dot')
    vz_dot = SX.sym('vz_dot')
    # 欧拉角
    phi_dot = SX.sym('phi_dot')     # roll
    theta_dot = SX.sym('theta_dot') # pitch
    psi_dot = SX.sym('psi_dot')     # yaw
    # 角速度
    p_dot = SX.sym('p_dot')     # wx
    q_dot = SX.sym('q_dot')     # wy
    r_dot = SX.sym('r_dot')     # wz
    states_dot = vertcat(x_dot, y_dot, z_dot, vx_dot, vy_dot, vz_dot, phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot)

    # 位置动力学
    x_d = vx
    y_d = vy
    z_d = vz
    vx_d = (f/m) * (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi))
    vy_d = (f/m) * (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))
    vz_d = - g + (f/m) * cos(phi)*cos(theta)
    # 欧拉角导数
    phi_d = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
    theta_d = q*cos(phi) - r*sin(phi)
    psi_d =  q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)
    # 姿态动力学
    p_d = (1/Ixx) * (tau_x + q*r*(Iyy-Izz) - J_RP*q*Omega)
    q_d = (1/Iyy) * (tau_y + p*r*(Izz-Ixx) + J_RP*p*Omega)
    r_d = (1/Izz) * (tau_z + p*q*(Ixx-Iyy))

    # ? 构建显式表达式和隐式表达式（显式？隐式？） 
    f_expl = vertcat(x_d, y_d, z_d, vx_d, vy_d, vz_d, phi_d, theta_d, psi_d, p_d, q_d, r_d)
    f_impl = states_dot - f_expl

    # ? algebraic variables
    z = []
    # ? parameters
    p = []

    model = AcadosModel()
    model.f_impl_expr = f_impl
    model.f_expl_expr = f_expl
    model.x = states
    model.xdot = states_dot
    model.u = controls
    model.z = z
    model.p = p
    model.name = model_name
    return model

def setup_nmpc():
    ocp = AcadosOcp()
    # 定义模型
    model = ocp.model
    states, controls, rhs = quadrotor_model()
    model.x = states
    model.u = controls
    model.f_expl_expr = rhs
    # 设置 OCP 维度
    nx = states.size1()
    nu = controls.size1()
    N = 20  # 预测步长
    Tf = 2.0  # 预测时间
    ocp.dims.N = N
    
    # 设置成本函数
    Q = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # 状态权重
    R = np.diag([0.1, 0.1, 0.1, 0.1])  # 控制权重
    ocp.cost.W = np.block([[Q, np.zeros((nx, nu))], [np.zeros((nu, nx)), R]])
    ocp.cost.W_e = Q
    x_ref = np.zeros(nx)  # 目标状态
    u_ref = np.zeros(nu)  # 目标控制输入
    ocp.cost.cost_type = 'LINEAR_LS'
    ocp.cost.cost_type_e = 'LINEAR_LS'
    ocp.model.cost_y_expr = vertcat(states, controls)
    ocp.model.cost_y_expr_e = states
    ocp.cost.yref = np.concatenate((x_ref, u_ref))
    ocp.cost.yref_e = x_ref
    # 设置约束
    ocp.constraints.lbu = np.array([0.0, -1.0, -1.0, -1.0])  # 推力 f 必须为正
    ocp.constraints.ubu = np.array([10.0, 1.0, 1.0, 1.0])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])
    # 设置初始条件
    ocp.constraints.x0 = np.zeros(nx)
    # 设置求解器选项
    ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_cond_N = N
    ocp.solver_options.tf = Tf
    # 创建求解器
    solver = AcadosOcpSolver(ocp)
    return solver
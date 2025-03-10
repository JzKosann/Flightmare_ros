# 四旋翼NMPC控制器 20250304 Wakkk
# ACADOS NMPC
from acados_template import AcadosOcp, AcadosOcpSolver
from export_model import *
import numpy as np
import scipy.linalg
from os.path import dirname, join, abspath
import time
import matplotlib.pyplot as plt
# np.set_printoptions(precision=3)  # 设置精度
np.set_printoptions(suppress=True)  # 禁用科学计数法输出

# ACADOS NMPC控制器
class NMPC_Controller:
    def __init__(self):
        self.ocp = AcadosOcp()       # OCP 优化问题
        self.model = export_model()  # 导出四旋翼物理模型

        self.Tf = 0.75                      # 预测时间长度(s)
        self.N = 50                         # 预测步数(节点数量)
        self.nx = self.model.x.size()[0]    # 状态维度 13维度
        self.nu = self.model.u.size()[0]    # 控制输入维度 4维度
        self.ny = self.nx + self.nu         # 评估维度
        self.ny_e = self.nx                 # 终端评估维度

        # set ocp_nlp_dimensions
        self.nlp_dims     = self.ocp.dims
        self.nlp_dims.N   = self.N

        # parameters
        self.g0  = 9.81    # [m.s^2] accerelation of gravity
        self.mq  = 1.0     # [kg] total mass (with one marker)
        self.Ct  = 1.33e-06   # [N/krpm^2] Thrust coef

        # bounds
        self.hov_w = np.sqrt((self.mq*self.g0)/(4*self.Ct))  # 悬停时单个电机推力
        print(f"hovor speed: {self.hov_w} krpm")
        # hovor speed: 15.777730167256925 krpm

        self.max_speed = 2e03  # 电机最高转速(krpm)

        # set weighting matrices 状态权重矩阵
        Q = np.eye(self.nx)
        Q[0,0] = 100.0      # x
        Q[1,1] = 100.0      # y
        Q[2,2] = 200.0      # z
        Q[3,3] = 0.0        # qw
        Q[4,4] = 0.0        # qx
        Q[5,5] = 0.0        # qy
        Q[6,6] = 0.0        # qz
        Q[7,7] = 1.0        # vbx
        Q[8,8] = 1.0        # vby
        Q[9,9] = 4.0        # vbz
        Q[10,10] = 1e-5     # wx
        Q[11,11] = 1e-5     # wy
        Q[12,12] = 10.0     # wz

        R = np.eye(self.nu)   # 控制输入权重矩阵
        R[0,0] = 0.06    # w1
        R[1,1] = 0.06    # w2
        R[2,2] = 0.06    # w3
        R[3,3] = 0.06    # w4

        self.ocp.cost.W = scipy.linalg.block_diag(Q, R)

        Vx = np.zeros((self.ny, self.nx))
        Vx[0,0] = 1.0
        Vx[1,1] = 1.0
        Vx[2,2] = 1.0
        Vx[3,3] = 1.0
        Vx[4,4] = 1.0
        Vx[5,5] = 1.0
        Vx[6,6] = 1.0
        Vx[7,7] = 1.0
        Vx[8,8] = 1.0
        Vx[9,9] = 1.0
        Vx[10,10] = 1.0
        Vx[11,11] = 1.0
        Vx[12,12] = 1.0
        self.ocp.cost.Vx = Vx

        Vu = np.zeros((self.ny, self.nu))
        Vu[13,0] = 1.0
        Vu[14,1] = 1.0
        Vu[15,2] = 1.0
        Vu[16,3] = 1.0
        self.ocp.cost.Vu = Vu

        self.ocp.cost.W_e = 50.0 * Q

        Vx_e = np.zeros((self.ny_e, self.nx))
        Vx_e[0,0] = 1.0
        Vx_e[1,1] = 1.0
        Vx_e[2,2] = 1.0
        Vx_e[3,3] = 1.0
        Vx_e[4,4] = 1.0
        Vx_e[5,5] = 1.0
        Vx_e[6,6] = 1.0
        Vx_e[7,7] = 1.0
        Vx_e[8,8] = 1.0
        Vx_e[9,9] = 1.0
        Vx_e[10,10] = 1.0
        Vx_e[11,11] = 1.0
        Vx_e[12,12] = 1.0
        self.ocp.cost.Vx_e = Vx_e

        # 过程参考向量(状态+输入)
        self.ocp.cost.yref   = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.hov_w, self.hov_w, self.hov_w, self.hov_w])
        # 终端参考向量(状态)
        self.ocp.cost.yref_e = np.array([0.0, 0.0, 0.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        # 构建约束
        self.ocp.constraints.lbu = np.array([0.0, 0.0, 0.0, 0.0])  # 电机最低转速输入
        self.ocp.constraints.ubu = np.array([+self.max_speed,+self.max_speed,+self.max_speed,+self.max_speed])  # 电机最高转速输入
        self.ocp.constraints.x0  = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 初始状态
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3])  # 所有电机转速参与评估

        # ocp.solver_options.qp_solver = 'FULL_CONDENSING_QPOASES'
        # self.ocp.solver_options.qp_solver = 'FULL_CONDENSING_HPIPM'  
        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'  
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0

        # set prediction horizon
        self.ocp.solver_options.tf = self.Tf
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # 显然更快 ~100Hz
        # self.ocp.solver_options.nlp_solver_type = 'SQP'  # ~10Hz

        self.ocp.model = self.model  # 传入模型
        # 构建编译OCP求解器
        self.acados_solver = AcadosOcpSolver(self.ocp, json_file = 'acados_ocp.json')
        print("NMPC Controller Init Done")

    # 状态空间位点控制
    # current_state当前状态: [x, y, z, qw, qx, qy, qz, vbx, vby, vbz, wx, wy, wz] 
    # goal_state目标状态:    [x, y, z, qw, qx, qy, qz, vbx, vby, vbz, wx, wy, wz] 
    def nmpc_state_control(self, current_state, goal_state):
        _start = time.perf_counter()
        # Set initial condition, equality constraint
        self.acados_solver.set(0, 'lbx', current_state)
        self.acados_solver.set(0, 'ubx', current_state)

        y_ref = np.concatenate((goal_state, np.array([self.hov_w, self.hov_w, self.hov_w, self.hov_w])))
        # Set Goal State
        for i in range(self.N):
            self.acados_solver.set(i, 'yref', y_ref)   # 过程参考
        y_refN = goal_state 
        self.acados_solver.set(self.N, 'yref', y_refN)   # 终端参考

        # Solve Problem
        self.acados_solver.solve()
        # Get Solution
        w_opt_acados = np.ndarray((self.N, 4))  # 控制输入
        x_opt_acados = np.ndarray((self.N + 1, len(current_state)))   # 状态估计
        x_opt_acados[0, :] = self.acados_solver.get(0, "x")
        for i in range(self.N):
            w_opt_acados[i, :] = self.acados_solver.get(i, "u")
            x_opt_acados[i + 1, :] = self.acados_solver.get(i + 1, "x")
        # return w_opt_acados, x_opt_acados  # 返回控制输入和状态
        _end = time.perf_counter()
        _dt = _end - _start
        return _dt, w_opt_acados[0], x_opt_acados  # 返回最近控制输入 4 Vector
        # control_input = self.acados_solver.get(0, "u")
        # state_estimate = self.acados_solver.get(self.N, "x")
        # return control_input, state_estimate  # 返回所有控制输入和状态

    # NMPC位置控制
    # goal_pos: 目标三维位置[x y z]
    def nmpc_position_control(self, current_state, goal_pos):
        goal_state = np.array([goal_pos[0], goal_pos[1], goal_pos[2], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        _dt, control = self.nmpc_state_control(current_state, goal_state)
        return _dt, control

# TEST
if __name__ == '__main__':
    print("202503010 ACADOS NMPC TEST")
    nmpc_controller = NMPC_Controller()
    current_state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    goal_state = np.array([0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    _dt, w, x_opt_acados = nmpc_controller.nmpc_state_control(current_state, goal_state)

    print("Control Input:")
    print(w)
    # print("State Estimation:")
    # print(x)
    time_steps = np.linspace(0, nmpc_controller.Tf, nmpc_controller.N + 1)

    plt.figure(figsize=(12, 8))

    # 位置 (x, y, z)
    plt.subplot(3, 1, 1)
    plt.plot(time_steps, x_opt_acados[:, 0], label='x')
    plt.plot(time_steps, x_opt_acados[:, 1], label='y')
    plt.plot(time_steps, x_opt_acados[:, 2], label='z')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.title('Position Trajectory')
    plt.legend()
    plt.grid(True)

    # 速度 (vx, vy, vz)
    plt.subplot(3, 1, 2)
    plt.plot(time_steps, x_opt_acados[:, 3], label='vx')
    plt.plot(time_steps, x_opt_acados[:, 4], label='vy')
    plt.plot(time_steps, x_opt_acados[:, 5], label='vz')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity')
    plt.title('Velocity Trajectory')
    plt.legend()
    plt.grid(True)

    # 欧拉角 (phi, theta, psi)
    plt.subplot(3, 1, 3)
    plt.plot(time_steps, x_opt_acados[:, 6], label='phi')
    plt.plot(time_steps, x_opt_acados[:, 7], label='theta')
    plt.plot(time_steps, x_opt_acados[:, 8], label='psi')
    plt.xlabel('Time (s)')
    plt.ylabel('Euler Angles')
    plt.title('Euler Angles Trajectory')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


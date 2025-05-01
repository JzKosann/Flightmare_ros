#

from acados_template import AcadosOcp, AcadosOcpSolver
from model import *
import scipy.linalg
import time
import matplotlib.pyplot as plt

    # parameter
m = 1.0  # 质量
g = 9.81  # 重力加速度
Ixx = 0.0234  # 转动惯量
Iyy = 0.0234
Izz = 0.0365
J_RP = 0.01  # 旋翼转动惯量
d = 0.25 # 旋翼到无人机中心的距离（臂长）
Cm = 2.128e-08 # 转矩系数
Ct = 1.33e-06 # 拉力系数

np.set_printoptions(suppress=True)

class NMPC_Controller:
    def __init__(self):
        self.ocp = AcadosOcp() # OCP优化问题
        self.model = quadrotor_model(m, g, Ixx, Iyy, Izz,
                                     J_RP, d, Cm, Ct)

        self.Tf = 1                       # 预测时间长度
        self.N = 50                         # 预测步数（节点数量）
        self.nx = self.model.x.size()[0]    # 状态维度 四元数 13
        self.nu = self.model.u.size()[0]    # 控制输入维度
        self.ny = self.nx + self.nu         # 评估维度 12+4 四元数 13+4
        self.ny_e = self.nx                 # 终端评估维度（终端误差）

        # set ocp_npl_dimensions
        self.nlp_dims   = self.ocp.dims
        self.nlp_dims.N = self.N

        # parameters
        self.g = g
        self.mq = m
        self.Ct = Ct 

        # bound
        self.hov_w = np.sqrt((self.mq*self.g)/(4*self.Ct))  # 悬停时单个电机转速
        hov_thrust = self.hov_w**2 * self.Ct
        print(f"hovor speed: {self.hov_w} rad/s , hover thrust: {hov_thrust} N")

        # 速度限制 單位rad/s
        self.max_speed = 2e+03
        self.min_speed = 150

        # 权重设置 # // TODO 权重需要更改
        # Q = np.eye(self.nx) # 状态权重
        Q = np.diag([   1000.0,  1000.0,  2000.0, 
                        50,      50,     50,
                        2000,      2000,      2000,    2000,
                        10,      10,    10])        
        R = np.eye(self.nu) # 控制权重
        # R = np.diag([5e-4, 5e-4, 5e-4, 5e-4])
        R = np.diag([0.01, 0.01, 0.01, 0.01])
        self.ocp.cost.W = scipy.linalg.block_diag(Q, R)

        Vx = np.zeros((self.ny, self.nx))
        Vx[:self.nx, :self.nx] = np.eye(self.nx)
        self.ocp.cost.Vx = Vx

        Vu = np.zeros((self.ny, self.nu))
        Vu[self.nx, 0] = 1.0
        Vu[self.nx+1, 1] = 1.0
        Vu[self.nx+2, 2] = 1.0
        Vu[self.nx+3, 3] = 1.0
        self.ocp.cost.Vu = Vu

        # self.ocp.cost.W_e = 50.0 * Q
        self.ocp.cost.W_e = 5*Q

        Vx_e = np.zeros((self.ny_e, self.nx))
        Vx_e = np.diag([    1.0,    1.0,    1.0,
                            1.0,    1.0,    1.0,
                            1.0,    1.0,    1.0,    1.0,
                            1.0,    1.0,    1.0,])
        self.ocp.cost.Vx_e = Vx_e

        # 过程参考向量(状态+输入) ref 这里是参考值初始化
        self.ocp.cost.yref = np.array([
                0.0, 0.0, 0.0,          # 位置 (x, y, z)
                0.0, 0.0, 0.0,          # 速度 (vx, vy, vz)
                1.0, 0.0, 0.0,  0.0,    # 四元数 (qw, qx ,qy, qz)
                # 0.0, 0.0, 0.0,          # 欧拉角 (phi, theta, psi)
                0.0, 0.0, 0.0,   # 角速度 (p, q, r)
                self.hov_w, self.hov_w, self.hov_w, self.hov_w  # 输入 (w1, w2, w3, w4)
                ])
        # 终端参考向量(状态)
        self.ocp.cost.yref_e = np.array([
                0.0, 0.0, 0.0,          # 位置 (x, y, z)
                0.0, 0.0, 0.0,          # 速度 (vx, vy, vz)
                1.0, 0.0, 0.0,  0.0,    # 四元数 (qw, qx ,qy, qz)
                # 0.0, 0.0, 0.0,          # 欧拉角 (phi, theta, psi)
                0.0, 0.0, 0.0,   # 角速度 (p, q, r)
                ])
        
        # 构建约束
        # 电机最低转速输入
        self.ocp.constraints.lbu = np.array([+self.min_speed,+self.min_speed,+self.min_speed,+self.min_speed])  
        # 电机最高转速输入
        self.ocp.constraints.ubu = np.array([+self.max_speed,+self.max_speed,+self.max_speed,+self.max_speed])  
        # 初始状态
        self.ocp.constraints.x0  = np.array([
                0.0, 0.0, 0.0,          # 位置 (x, y, z)
                0.0, 0.0, 0.0,          # 速度 (vx, vy, vz)
                1.0, 0.0, 0.0,  0.0,    # 四元数 (qw, qx ,qy, qz)
                # 0.0, 0.0, 0.0,          # 欧拉角 (phi, theta, psi)
                0.0, 0.0, 0.0,   # 角速度 (p, q, r)
        ])  
        # 所有电机转速参与评估
        self.ocp.constraints.idxbu = np.array([0, 1, 2, 3])  

        self.ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        self.ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        self.ocp.solver_options.integrator_type = 'ERK'
        self.ocp.solver_options.print_level = 0

        # set prediction horizon
        self.ocp.solver_options.tf = self.Tf
        self.ocp.solver_options.nlp_solver_type = 'SQP_RTI' # 100hz

        # set model
        self.ocp.model = self.model

        self.acados_solver = AcadosOcpSolver(self.ocp, json_file= f'acados_ocp_{self.model.name}.json')
        print("NMPC Controller Init Done!")

    # 状态空间位点控制
    # // current_state,  target_state:    [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r] 
    # current_state,  target_state:    [x, y, z, vx, vy, vz, qw, qx, qy, qz, p, q, r] 
    def nmpc_state_control(self, current_state, target_state):
        _start = time.perf_counter()
        # Set initial condition, equality constraint
        self.acados_solver.set(0, 'lbx', current_state)
        self.acados_solver.set(0, 'ubx', current_state)
        y_ref = np.concatenate((target_state, 
                                np.array([self.hov_w, self.hov_w, self.hov_w, self.hov_w])))
        # Set Goal State
        for i in range(self.N):
            self.acados_solver.set(i, 'yref', y_ref)   # 过程参考
        y_refN = target_state 
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
        _end = time.perf_counter() # 计算时间
        _dt = _end - _start
        f = Ct * (w_opt_acados[:, 0]**2 + w_opt_acados[:, 1]**2 + w_opt_acados[:, 2]**2 + w_opt_acados[:, 3]**2)
        # print("Thrust (f):", f)
        # print(w_opt_acados)
        print(f"控制转速： {w_opt_acados[0]}")
        controls = w_opt_acados[0]**2 * self.Ct
        # return _dt, w_opt_acados[0], x_opt_acados  # 返回最近控制输入 4 Vector(速度)
        return _dt, controls, x_opt_acados  # 返回最近控制输入 4 Vector(速度)
    
        # NMPC位置控制
    # goal_pos: 目标三维位置[x y z yaw]
    def nmpc_position_control(self, current_state, goal_pos):
        target_state = np.array([goal_pos[0], goal_pos[1], goal_pos[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        _dt, control, x_opt_acados = self.nmpc_state_control(current_state, target_state)
        return _dt, control
    


# TEST 
if __name__ == '__main__':
    
    nmpc_controller = NMPC_Controller()
    cur_state = np.array([0,0,1,0,0,0,0,0,0,0,0,0,0])
    tar_pos = np.array([0,0,20,0,0,0,0,0,0,0,0,0,0])
    _dt, w,x_opt_acados = nmpc_controller.nmpc_state_control(cur_state, tar_pos)

    print(x_opt_acados)
        # 绘制状态轨迹
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

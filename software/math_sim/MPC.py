import numpy as np
import cvxopt
from cvxopt import matrix, solvers
import matplotlib.pyplot as plt

# 关闭求解器详细输出
solvers.options['show_progress'] = False

class DroneMPC:
    """
    四旋翼俯仰通道 MPC 控制器
    状态 x = [theta, q]^T  (角度, 角速度)
    输入 u = tau_pitch     (俯仰力矩)
    """
    def __init__(self, dt=0.05, N=10):
        self.dt = dt          # 采样时间 (50Hz 典型位置环)
        self.N = N            # 预测时域长度
        
        # 系统参数
        Iyy = 0.01            # 俯仰转动惯量 kg·m²
        
        # 连续状态空间模型
        # theta_dot = q
        # q_dot = u / Iyy
        self.A_c = np.array([[0, 1],
                             [0, 0]])
        self.B_c = np.array([[0],
                             [1/Iyy]])
        
        # 离散化 (前向欧拉，适合 50Hz)
        self.A_d = np.eye(2) + self.A_c * self.dt
        self.B_d = self.B_c * self.dt
        
        self.nx = 2           # 状态维度
        self.nu = 1           # 控制维度
        
        # 代价权重
        self.Q = np.diag([100.0, 1.0])   # 角度权重 >> 角速度权重
        self.R = np.diag([0.1])          # 控制量权重
        
        # 执行器物理约束 (力矩饱和)
        self.u_max =  2.0      # N·m
        self.u_min = -2.0
        
        # 构建 QP 矩阵
        self._build_qp_matrices()
        
    def _build_qp_matrices(self):
        """
        构建 MPC 二次规划的常数矩阵
        MPC 问题转化为:
            min_U   U^T H U + 2 x0^T F^T U
            s.t.    G U <= h
        """
        # ---------- 预测模型矩阵 ----------
        # X = A_qp * x0 + B_qp * U
        # U = [u_0, u_1, ..., u_{N-1}]^T
        
        A_qp = np.zeros((self.nx * (self.N + 1), self.nx))
        B_qp = np.zeros((self.nx * (self.N + 1), self.nu * self.N))
        
        A_pow = np.eye(self.nx)
        for i in range(self.N + 1):
            A_qp[i*self.nx:(i+1)*self.nx, :] = A_pow
            for j in range(self.N):
                if i > j:
                    power = i - j - 1
                    B_qp[i*self.nx:(i+1)*self.nx, j*self.nu:(j+1)*self.nu] = \
                        np.linalg.matrix_power(self.A_d, power) @ self.B_d
            A_pow = A_pow @ self.A_d
            
        # ---------- 代价函数矩阵 ----------
        # 增广权重矩阵
        Q_bar = np.kron(np.eye(self.N), self.Q)
        Q_bar = np.block([
            [Q_bar, np.zeros((self.N*self.nx, self.nx))],
            [np.zeros((self.nx, self.N*self.nx)), self.Q]
        ])
        R_bar = np.kron(np.eye(self.N), self.R)
        
        # 去掉初始状态块
        B_ctrl = B_qp[self.nx:, :]
        Q_ctrl = Q_bar[self.nx:, self.nx:]
        A_ctrl = A_qp[self.nx:, :]
        
        # H = 2 * (B^T Q B + R)
        self.H = 2 * (B_ctrl.T @ Q_ctrl @ B_ctrl + R_bar)
        # F = 2 * (A^T Q B)^T
        self.F = 2 * (B_ctrl.T @ Q_ctrl @ A_ctrl)
        
        # ---------- 约束矩阵 ----------
        # u_min <= u_k <= u_max
        G = np.vstack((
            np.eye(self.N * self.nu),
            -np.eye(self.N * self.nu)
        ))
        h = np.hstack((
            self.u_max * np.ones(self.N * self.nu),
            -self.u_min * np.ones(self.N * self.nu)
        ))
        
        # 转换为 CVXOPT 格式
        self.H_cvx = matrix(self.H)
        self.G_cvx = matrix(G)
        self.h_cvx = matrix(h)
        
        # 保存用于预测轨迹展示
        self.A_qp_full = A_qp
        self.B_qp_full = B_qp

    def solve(self, x_current):
        """
        求解 MPC 优化问题
        返回: u_mpc (第一个控制量), U_opt (整个最优序列)
        """
        x0 = np.array(x_current).flatten()
        
        # 梯度向量: F_x0 = F @ x0
        F_x0 = self.F @ x0
        
        # 求解二次规划
        sol = solvers.qp(self.H_cvx, matrix(F_x0), self.G_cvx, self.h_cvx)
        
        U_opt = np.array(sol['x']).flatten()
        u_mpc = np.clip(U_opt[0], self.u_min, self.u_max)
        
        return u_mpc, U_opt

    def predict_trajectory(self, x_current, U_opt):
        """根据最优控制序列预测未来状态轨迹"""
        x0 = np.array(x_current).flatten()
        X_pred = self.A_qp_full @ x0 + self.B_qp_full @ U_opt
        return X_pred.reshape(self.N + 1, self.nx)

    def simulate(self, initial_state, steps=80):
        """
        闭环仿真
        initial_state: [theta_0, q_0]
        """
        x = np.array(initial_state).reshape(-1, 1)
        
        history_x = [x.flatten()]
        history_u = []
        
        for _ in range(steps):
            # 求解 MPC
            u, _ = self.solve(x.flatten())

            # 状态更新 (B_d 是 2x1, u 需要转成列向量)
            x = self.A_d @ x + self.B_d @ np.array([[u]])

            history_x.append(x.flatten())
            history_u.append(u)
            
        return np.array(history_x), np.array(history_u)


# ---------- 仿真主程序 ----------
def run_mpc_simulation():
    # 控制器参数
    dt = 0.05      # 50Hz
    N = 10         # 预测时域 0.5 秒
    
    mpc = DroneMPC(dt=dt, N=N)
    
    # 初始状态：偏离水平 45 度 (模拟强扰动)
    initial_state = [np.radians(45), 0.0]
    
    print(f"MPC 控制器已初始化")
    print(f"采样时间: {dt}s, 预测时域: {N} 步 ({N*dt}s)")
    print(f"初始俯仰角: {np.degrees(initial_state[0]):.1f}°")
    print("\n运行仿真中...")
    
    # 仿真 4 秒
    steps = int(4.0 / dt)
    states, controls = mpc.simulate(initial_state, steps)
    
    # ---------- Visualization ----------
    time = np.arange(len(states)) * dt
    time_u = time[:-1]

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # Plot 1: Pitch Angle Response
    ax1 = axes[0, 0]
    ax1.plot(time, np.degrees(states[:, 0]), 'b-', linewidth=2)
    ax1.axhline(0, color='gray', linestyle='--', alpha=0.7)
    ax1.set_ylabel('Pitch Angle (deg)')
    ax1.set_xlabel('Time (s)')
    ax1.grid(True)
    ax1.set_title('Pitch Angle Response')

    # Plot 2: Angular Velocity Response
    ax2 = axes[0, 1]
    ax2.plot(time, np.degrees(states[:, 1]), 'r-', linewidth=2)
    ax2.axhline(0, color='gray', linestyle='--', alpha=0.7)
    ax2.set_ylabel('Pitch Rate (deg/s)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True)
    ax2.set_title('Angular Velocity Response')

    # Plot 3: Control Input
    ax3 = axes[1, 0]
    ax3.step(time_u, controls, 'g-', linewidth=2, where='post')
    ax3.axhline(mpc.u_max, color='k', linestyle=':', label=f'Limit: +/-{mpc.u_max} N·m')
    ax3.axhline(-mpc.u_max, color='k', linestyle=':')
    ax3.set_ylabel('Torque (N·m)')
    ax3.set_xlabel('Time (s)')
    ax3.legend()
    ax3.grid(True)
    ax3.set_title('Control Input')

    # Plot 4: Phase Portrait
    ax4 = axes[1, 1]
    ax4.plot(np.degrees(states[:, 0]), np.degrees(states[:, 1]), 'm-', linewidth=2)
    ax4.plot(np.degrees(states[0, 0]), np.degrees(states[0, 1]), 'go', label='Start')
    ax4.plot(0, 0, 'r*', markersize=12, label='Target')
    ax4.set_xlabel('Pitch Angle (deg)')
    ax4.set_ylabel('Pitch Rate (deg/s)')
    ax4.legend()
    ax4.grid(True)
    ax4.set_title('Phase Portrait')

    plt.suptitle(f'MPC Pitch Control Simulation (Horizon N={N}, dt={dt}s)')
    plt.tight_layout()
    plt.show()
    
    # ---------- 性能指标 ----------
    print("\n=== 性能指标 ===")
    print(f"调节时间 (进入 ±1° 范围): ", end="")
    settled_idx = np.where(np.abs(states[:, 0]) < np.radians(1))[0]
    if len(settled_idx) > 0:
        settle_time = settled_idx[0] * dt
        print(f"{settle_time:.2f} s")
    else:
        print("未收敛")
    
    print(f"最大角速度: {np.max(np.abs(np.degrees(states[:, 1]))):.1f} deg/s")
    print(f"控制量 RMS: {np.sqrt(np.mean(controls**2)):.3f} N·m")
    print(f"总控制能量: {np.sum(controls**2) * dt:.3f} N²·m²·s")


if __name__ == "__main__":
    run_mpc_simulation()
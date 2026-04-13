import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import solve_discrete_are
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque
import random

# 设置随机种子保证可复现
np.random.seed(42)
torch.manual_seed(42)

# ==================== 1. 真实四旋翼动力学模型（带未建模动态） ====================
class TrueQuadrotorDynamics:
    """
    真实的四旋翼俯仰通道动力学（包含 MPC 不知道的非线性效应）
    状态: x = [theta, q]  (角度, 角速度)
    输入: u = 电机力矩
    """
    def __init__(self, dt=0.02):
        self.dt = dt
        self.Iyy_nom = 0.015           # 名义转动惯量（MPC 知道的）
        self.Iyy_true = 0.022          # 真实转动惯量（挂载后变大了）
        self.damping_nom = 0.0         # 名义阻尼
        self.damping_true = -0.15      # 真实气动阻尼（MPC 不知道）
        self.gravity_term = 0.0        # 重力分量
        
    def set_payload(self, mass_ratio):
        """模拟挂载负载导致转动惯量增加"""
        self.Iyy_true = self.Iyy_nom * (1 + mass_ratio)
        
    def forward(self, x, u):
        """真实动力学一步前向"""
        theta, q = x
        
        # 真实动力学方程（包含未知项）
        q_dot = (u / self.Iyy_true) + self.damping_true * q
        theta_dot = q
        
        # 欧拉积分
        theta_next = theta + theta_dot * self.dt
        q_next = q + q_dot * self.dt
        
        return np.array([theta_next, q_next])

# ==================== 2. 神经网络残差学习器 ====================
class ResidualLearner(nn.Module):
    """
    学习名义模型与真实动力学之间的残差
    输入: [theta, q, u]   (当前状态 + 控制量)
    输出: [delta_theta, delta_q]  (残差修正)
    """
    def __init__(self, hidden_dim=64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(3, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 2)
        )
        
    def forward(self, x_u):
        return self.net(x_u)

class ReplayBuffer:
    """经验回放缓冲区"""
    def __init__(self, capacity=1000):
        self.buffer = deque(maxlen=capacity)
        
    def push(self, x, u, x_next):
        """存储一次状态转移"""
        self.buffer.append((x.copy(), u, x_next.copy()))
        
    def sample(self, batch_size):
        """随机采样一批数据"""
        batch = random.sample(self.buffer, min(batch_size, len(self.buffer)))
        x_batch = np.array([item[0] for item in batch])
        u_batch = np.array([item[1] for item in batch]).reshape(-1, 1)
        x_next_batch = np.array([item[2] for item in batch])
        
        return x_batch, u_batch, x_next_batch
    
    def __len__(self):
        return len(self.buffer)

# ==================== 3. 学习型 MPC 控制器 ====================
class LearningMPC:
    """
    带有神经网络残差学习的 MPC 控制器
    """
    def __init__(self, dt=0.02, N=10):
        self.dt = dt
        self.N = N
        
        # 名义模型参数（MPC 内部使用的简化模型）
        self.Iyy = 0.015
        self.A_c = np.array([[0, 1],
                             [0, 0]])
        self.B_c = np.array([[0],
                             [1/self.Iyy]])
        
        # 离散化
        self.A_d = np.eye(2) + self.A_c * dt
        self.B_d = self.B_c * dt
        
        # 代价权重
        self.Q = np.diag([100.0, 1.0])
        self.R = np.diag([0.1])
        
        # 控制约束
        self.u_max = 3.0
        self.u_min = -3.0
        
        # 神经网络残差学习器
        self.residual_nn = ResidualLearner(hidden_dim=64)
        self.optimizer = optim.Adam(self.residual_nn.parameters(), lr=0.001)
        self.replay_buffer = ReplayBuffer(capacity=2000)
        self.use_learning = True  # 是否启用学习修正
        
        # 训练统计
        self.train_losses = []
        
    def predict_nominal(self, x, u):
        """名义模型预测（不考虑残差）"""
        return self.A_d @ x + self.B_d @ u
    
    def predict_with_residual(self, x, u):
        """带残差修正的模型预测"""
        # 名义预测
        x_nom = self.predict_nominal(x, u)
        
        if not self.use_learning:
            return x_nom
            
        # 神经网络残差预测
        self.residual_nn.eval()
        with torch.no_grad():
            x_u_input = torch.FloatTensor(np.hstack([x, u]).reshape(1, -1))
            residual = self.residual_nn(x_u_input).numpy().flatten()
            
        return x_nom + residual
    
    def train_residual_model(self, batch_size=64, epochs=10):
        """训练神经网络残差模型"""
        if len(self.replay_buffer) < batch_size:
            return
            
        self.residual_nn.train()
        epoch_losses = []
        
        for epoch in range(epochs):
            x_batch, u_batch, x_next_batch = self.replay_buffer.sample(batch_size)
            
            # 计算名义模型预测
            x_next_nom_batch = np.zeros_like(x_batch)
            for i in range(len(x_batch)):
                x_next_nom_batch[i] = self.predict_nominal(x_batch[i], u_batch[i])
            
            # 真实残差
            true_residual = x_next_batch - x_next_nom_batch
            
            # 准备网络输入
            inputs = np.hstack([x_batch, u_batch])
            inputs_tensor = torch.FloatTensor(inputs)
            true_residual_tensor = torch.FloatTensor(true_residual)
            
            # 前向传播
            pred_residual = self.residual_nn(inputs_tensor)
            loss = nn.MSELoss()(pred_residual, true_residual_tensor)
            
            # 反向传播
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
            
            epoch_losses.append(loss.item())
            
        self.train_losses.append(np.mean(epoch_losses))
        
    def solve_mpc(self, x_current):
        """
        简化 MPC 求解（一维控制，可直接用 LQR 近似 + 裁剪）
        真实 MPC 应解 QP，这里用解析 LQR 简化以突出学习部分
        """
        # 计算 LQR 增益
        P = solve_discrete_are(self.A_d, self.B_d, self.Q, self.R)
        K = np.linalg.inv(self.R + self.B_d.T @ P @ self.B_d) @ (self.B_d.T @ P @ self.A_d)
        
        # 反馈控制
        u = -K @ x_current
        
        # 约束裁剪
        u = np.clip(u, self.u_min, self.u_max)
        
        return u.item()
    
    def control(self, x_current, train=True):
        """主控制接口"""
        u = self.solve_mpc(x_current)
        
        # 收集数据用于训练
        if train and hasattr(self, 'last_x') and hasattr(self, 'last_u'):
            self.replay_buffer.push(self.last_x, self.last_u, x_current)
            
        # 保存当前状态用于下一次
        self.last_x = x_current.copy()
        self.last_u = u
        
        return u

# ==================== 4. 仿真对比实验 ====================
def run_comparison_experiment():
    """对比标准 MPC 和学习型 MPC 在模型失配下的表现"""
    
    dt = 0.02
    sim_time = 8.0
    steps = int(sim_time / dt)
    
    # 真实动力学（带模型误差）
    true_dynamics = TrueQuadrotorDynamics(dt=dt)
    true_dynamics.set_payload(mass_ratio=0.5)  # 挂载导致转动惯量增加 50%
    
    # 创建两个控制器
    standard_mpc = LearningMPC(dt=dt, N=10)
    standard_mpc.use_learning = False  # 关闭学习功能
    
    learning_mpc = LearningMPC(dt=dt, N=10)
    learning_mpc.use_learning = True   # 开启学习功能
    
    # 初始状态（45度偏角）
    x0 = np.array([np.radians(45), 0.0])
    
    # 目标状态
    x_target = np.array([0.0, 0.0])
    
    # 数据记录
    history_std = {'x': [x0.copy()], 'u': [], 'theta': [np.degrees(x0[0])]}
    history_lrn = {'x': [x0.copy()], 'u': [], 'theta': [np.degrees(x0[0])]}
    
    # 仿真循环
    x_std = x0.copy()
    x_lrn = x0.copy()
    
    print("开始仿真对比实验...")
    print(f"真实转动惯量 = {true_dynamics.Iyy_true:.4f} (名义值 = {true_dynamics.Iyy_nom:.4f})")
    print(f"模型误差: {(true_dynamics.Iyy_true/true_dynamics.Iyy_nom - 1)*100:.1f}%")
    
    for step in range(steps):
        # 计算误差状态
        e_std = x_std - x_target
        e_lrn = x_lrn - x_target
        
        # 获取控制量
        u_std = standard_mpc.control(e_std, train=False)
        u_lrn = learning_mpc.control(e_lrn, train=True)
        
        # 执行动力学
        x_std_next = true_dynamics.forward(x_std, u_std)
        x_lrn_next = true_dynamics.forward(x_lrn, u_lrn)
        
        # 记录数据
        history_std['x'].append(x_std_next.copy())
        history_std['u'].append(u_std)
        history_std['theta'].append(np.degrees(x_std_next[0]))
        
        history_lrn['x'].append(x_lrn_next.copy())
        history_lrn['u'].append(u_lrn)
        history_lrn['theta'].append(np.degrees(x_lrn_next[0]))
        
        # 更新状态
        x_std = x_std_next
        x_lrn = x_lrn_next
        
        # 每 200 步训练一次神经网络
        if step % 50 == 0 and step > 100:
            learning_mpc.train_residual_model(batch_size=64, epochs=20)
            
        # 打印进度
        if step % 100 == 0:
            print(f"Step {step}/{steps}, "
                  f"Std Theta: {np.degrees(x_std[0]):6.2f}°, "
                  f"Lrn Theta: {np.degrees(x_lrn[0]):6.2f}°")
    
    # 最终训练
    print("\n最终训练神经网络...")
    for _ in range(5):
        learning_mpc.train_residual_model(batch_size=128, epochs=50)
    
    return history_std, history_lrn, learning_mpc

# ==================== 5. 可视化 ====================
def plot_results(history_std, history_lrn, lrn_mpc, dt=0.02):
    """绘制对比结果"""
    
    time = np.arange(len(history_std['theta'])) * dt
    time_u = time[:-1]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 8))

    # Plot 1: Pitch Angle Comparison
    ax1 = axes[0, 0]
    ax1.plot(time, history_std['theta'], 'r-', linewidth=2, label='Standard MPC')
    ax1.plot(time, history_lrn['theta'], 'b-', linewidth=2, label='Learning MPC')
    ax1.axhline(0, color='gray', linestyle='--', alpha=0.7)
    ax1.set_ylabel('Pitch Angle (deg)')
    ax1.set_xlabel('Time (s)')
    ax1.legend()
    ax1.grid(True)
    ax1.set_title('Pitch Angle Comparison (45 deg Initial)')

    # Plot 2: Angular Velocity Response
    ax2 = axes[0, 1]
    q_std = np.array([x[1] for x in history_std['x']])
    q_lrn = np.array([x[1] for x in history_lrn['x']])
    ax2.plot(time, np.degrees(q_std), 'r-', linewidth=2, label='Standard MPC')
    ax2.plot(time, np.degrees(q_lrn), 'b-', linewidth=2, label='Learning MPC')
    ax2.set_ylabel('Pitch Rate (deg/s)')
    ax2.set_xlabel('Time (s)')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Angular Velocity Response')

    # Plot 3: Control Input
    ax3 = axes[0, 2]
    ax3.step(time_u, history_std['u'], 'r-', linewidth=2, where='post', alpha=0.7)
    ax3.step(time_u, history_lrn['u'], 'b-', linewidth=2, where='post', alpha=0.7)
    ax3.axhline(3.0, color='k', linestyle=':', label='Limit')
    ax3.axhline(-3.0, color='k', linestyle=':')
    ax3.set_ylabel('Torque (N·m)')
    ax3.set_xlabel('Time (s)')
    ax3.legend(['Standard MPC', 'Learning MPC'])
    ax3.grid(True)
    ax3.set_title('Control Input')

    # Plot 4: Steady-State Error Zoom (Last 2s)
    ax4 = axes[1, 0]
    last_idx = int(2.0 / dt)
    time_zoom = time[-last_idx:]
    theta_std_zoom = history_std['theta'][-last_idx:]
    theta_lrn_zoom = history_lrn['theta'][-last_idx:]
    ax4.plot(time_zoom, theta_std_zoom, 'r-', linewidth=2)
    ax4.plot(time_zoom, theta_lrn_zoom, 'b-', linewidth=2)
    ax4.axhline(0, color='gray', linestyle='--', alpha=0.7)
    ax4.set_ylabel('Pitch Angle (deg)')
    ax4.set_xlabel('Time (s)')
    ax4.grid(True)
    ax4.set_title('Steady-State Error Zoom (Last 2s)')

    # Plot 5: Training Loss
    ax5 = axes[1, 1]
    if len(lrn_mpc.train_losses) > 0:
        ax5.plot(lrn_mpc.train_losses, 'g-', linewidth=2)
        ax5.set_ylabel('MSE Loss')
        ax5.set_xlabel('Training Epoch')
        ax5.grid(True)
        ax5.set_title('Neural Network Training Loss')
        ax5.set_yscale('log')

    # Plot 6: Cumulative Absolute Error
    ax6 = axes[1, 2]
    cum_error_std = np.cumsum(np.abs(np.array(history_std['theta']))) * dt
    cum_error_lrn = np.cumsum(np.abs(np.array(history_lrn['theta']))) * dt
    ax6.plot(time, cum_error_std, 'r-', linewidth=2)
    ax6.plot(time, cum_error_lrn, 'b-', linewidth=2)
    ax6.set_ylabel('Cumulative Abs Error (deg·s)')
    ax6.set_xlabel('Time (s)')
    ax6.legend(['Standard MPC', 'Learning MPC'])
    ax6.grid(True)
    ax6.set_title('Cumulative Tracking Error')

    plt.suptitle('Quadrotor Pitch Channel: Standard MPC vs Neural Learning MPC', fontsize=14)
    plt.tight_layout()
    plt.show()
    
    # 打印性能指标
    print("\n" + "="*50)
    print("性能指标对比")
    print("="*50)
    
    # 计算稳态误差 (最后1秒的平均值)
    steady_idx = int(1.0 / dt)
    steady_std = np.mean(np.abs(history_std['theta'][-steady_idx:]))
    steady_lrn = np.mean(np.abs(history_lrn['theta'][-steady_idx:]))
    
    print(f"稳态误差 (最后1秒均值):")
    print(f"  标准 MPC:   {steady_std:.4f}°")
    print(f"  学习型 MPC: {steady_lrn:.4f}°")
    print(f"  改善比例:   {(1 - steady_lrn/steady_std)*100:.1f}%")
    
    print(f"\n控制能量消耗 (∑|u|):")
    energy_std = np.sum(np.abs(history_std['u'])) * dt
    energy_lrn = np.sum(np.abs(history_lrn['u'])) * dt
    print(f"  标准 MPC:   {energy_std:.2f}")
    print(f"  学习型 MPC: {energy_lrn:.2f}")
    
    print(f"\n累计绝对误差:")
    print(f"  标准 MPC:   {cum_error_std[-1]:.2f} deg·s")
    print(f"  学习型 MPC: {cum_error_lrn[-1]:.2f} deg·s")

# ==================== 6. 主程序 ====================
if __name__ == "__main__":
    print("="*60)
    print("四旋翼神经网络学习型 MPC 仿真")
    print("="*60)
    
    # 运行对比实验
    history_std, history_lrn, lrn_mpc = run_comparison_experiment()
    
    # 绘制结果
    plot_results(history_std, history_lrn, lrn_mpc, dt=0.02)
    
    print("\n仿真完成!")
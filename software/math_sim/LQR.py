import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt

class DroneLQR:
    def __init__(self):
        # Physical parameters
        # Moment of inertia Ixx (kg*m^2)
        Ixx = 0.01
        # Sampling time (s)
        self.dt = 0.01

        # Continuous-time state space model: x_dot = A @ x + B @ u
        # x = [theta, q]^T - state: pitch angle (rad), angular velocity (rad/s)
        # u - input: motor torque difference (N*m)
        #
        # State equations:
        #   theta_dot = q
        #   q_dot = u / Ixx
        #
        # A = [0   1]     B = [  0  ]
        #     [0   0]         [1/Ixx]
        self.A = np.array([[0, 1],
                          [0, 0]])
        self.B = np.array([[0],
                          [1/Ixx]])

        # Euler forward discretization: x(k+1) = Ad @ x(k) + Bd @ u(k)
        # Ad = I + A * dt
        # Bd = B * dt
        self.Ad = np.eye(2) + self.A * self.dt
        self.Bd = self.B * self.dt

        # LQR weight matrices
        # Q: state penalty matrix (2x2 symmetric)
        # R: control input penalty matrix (1x1)
        self.Q = np.array([[100.0, 0.0],
                          [0.0, 1.0]])
        self.R = np.array([[0.1]])

        # Solve discrete-time LQR
        self.K = self.solve_dlqr(self.Ad, self.Bd, self.Q, self.R)
        print(f"Feedback gain K: {self.K}")

    def solve_dlqr(self, A, B, Q, R):
        # Solve Discrete-time Algebraic Riccati Equation (DARE)
        #
        # DARE: P = Q + A^T @ P @ A - A^T @ P @ B @ inv(R + B^T @ P @ B) @ B^T @ P @ A
        #
        # Optimal feedback gain: K = inv(R + B^T @ P @ B) @ B^T @ P @ A
        P = Q.copy()
        P_next = np.zeros_like(P)
        max_iter = 100
        tol = 1e-8

        for _ in range(max_iter):
            P_next = Q + A.T @ P @ A - A.T @ P @ B @ la.inv(R + B.T @ P @ B) @ B.T @ P @ A
            if la.norm(P_next - P) < tol:
                break
            P = P_next

        # K = (R + B^T P B)^{-1} B^T P A
        K = la.inv(R + B.T @ P @ B) @ B.T @ P @ A
        return K

    def simulate(self, initial_state, target_state, steps=500):
        # Closed-loop simulation
        # Control law: u = -K @ error, where error = x - x_target
        # State update: x = Ad @ x + Bd @ u
        x = np.array(initial_state).reshape(-1, 1)
        x_target = np.array(target_state).reshape(-1, 1)
        history = [x.flatten()]
        u_history = []

        for _ in range(steps):
            error = x - x_target
            u = -self.K @ error
            u = np.clip(u, -2.0, 2.0)
            x = self.Ad @ x + self.Bd @ u
            history.append(x.flatten())
            u_history.append(u.flatten()[0])

        return np.array(history), np.array(u_history)

drone = DroneLQR()

initial = [np.radians(30), 0]
target = [0, 0]

states, controls = drone.simulate(initial, target)

fig, axs = plt.subplots(2, 1, figsize=(10, 6))
time = np.arange(len(states)) * drone.dt

axs[0].plot(time, np.degrees(states[:, 0]), label='Pitch Angle (deg)')
axs[0].axhline(0, color='gray', linestyle='--')
axs[0].set_ylabel('Angle')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(time[:-1], controls, label='Control Input (Torque)', color='orange')
axs[1].set_xlabel('Time (s)')
axs[1].set_ylabel('Torque (N·m)')
axs[1].legend()
axs[1].grid(True)

plt.suptitle('UAV LQR Attitude Regulation Response')
plt.show()

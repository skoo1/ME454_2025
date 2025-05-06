import numpy as np
import matplotlib.pyplot as plt

# Physical constants and system parameters
m1= 0.20                    # mass [kg]
l1 = 0.50                   # link half length [m]
I1 = m1 * (2*l1)**2 / 12    # moment of inertia link 1 [kg·m²]
g = 9.81                    # gravity [m/s²]

# Initial angles (theta2 derived from closed-chain constraint)
theta1 = np.pi / 4  # [rad]

# Initial positions of link com
x1 =  l1 * np.sin(theta1)
y1 =  -l1 * np.cos(theta1)

# Initial velocities (stationary)
q     = np.array([x1, y1, theta1])
q_dot = np.zeros(3)

# Define mass-inertia matrix M
M = np.diag([m1, m1, I1])

# Baumgarte stabilization parameters (adjustable)
alpha = 10.0  # velocity stabilization gain
beta = 100.0  # position stabilization gain

def update_J_and_J_dot(q, q_dot):
    x1, y1, th1 = q
    vx1, vy1, w1 = q_dot

    # Constraint Jacobian J (2 constraints × 3 generalized coordinates)
    J = np.array([
        [ 1,  0, -l1*np.cos(th1)],
        [ 0,  1, -l1*np.sin(th1)]
    ])

    # Time derivative of J
    J_dot = np.array([
        [ 0, 0,  l1*w1*np.sin(th1)],
        [ 0, 0,  -l1*w1*np.cos(th1)]
    ])

    return J, J_dot


def update_F_ext():
    # Generalized external forces: [Fx1, Fy1, τ1]
    return np.array([0, -m1*g, 0])


def compute_q_ddot(q, q_dot, M, alpha, beta):
    J, J_dot = update_J_and_J_dot(q, q_dot)
    F_ext    = update_F_ext()

    # Constraint error (position)
    C = np.array([q[0] - l1 * np.sin(q[2]), q[1] + l1 * np.cos(q[2])])
    C_dot = np.dot(J, q_dot)

    # Construct b vector
    b_top    = F_ext
    b_bottom = -np.dot(J_dot, q_dot) - 2 * alpha * C_dot  - beta**2 * C  # Baumgarte stabilization term
    b = np.concatenate((b_top, b_bottom))

    # Construct A matrix
    A_top    = np.hstack((M, J.T))
    A_bottom = np.hstack((J, np.zeros((J.shape[0], J.shape[0]))))
    A = np.vstack((A_top, A_bottom))

    # Solve linear system for q̈ and Lagrange multipliers
    x = np.linalg.solve(A, b)
    q_ddot       = x[:3]
    lambda_prime = x[3:]

    return q_ddot, lambda_prime, C, C_dot


# Simulation parameters
dt    = 0.001    # time step [s]
T     = 2.0      # total simulation time [s]
steps = int(T/dt)

# For storing results
q_hist     = np.zeros((steps+1, 3))
q_dot_hist = np.zeros((steps+1, 3))
t_hist     = np.linspace(0, T, steps+1)

# For storing constraint violations
C_hist     = np.zeros(steps+1)  # C(q) violation
C_dot_hist = np.zeros(steps+1)  # C_dot(q) violation

# Record initial conditions
q_hist[0]     = q
q_dot_hist[0] = q_dot

# Main loop
for i in range(steps):
    q_ddot, _, C, C_dot = compute_q_ddot(q, q_dot, M, alpha, beta)  # Pass Baumgarte parameters here

    # Store constraint violations
    C_hist[i+1]     = np.linalg.norm(C)
    C_dot_hist[i+1] = np.linalg.norm(C_dot)

    # Semi-implicit Euler integration
    q_dot += q_ddot * dt
    q     += q_dot   * dt

    q_hist[i+1]     = q
    q_dot_hist[i+1] = q_dot


# Plot only 100 equally-spaced configurations
num_samples = 500
indices = np.linspace(0, len(q_hist)-1, num_samples, dtype=int)

plt.figure()
for idx in indices[::10]: # every 10th
    x1, y1, th1 = q_hist[idx]

    # Link 1 endpoints (from center at x1, y1)
    dx1 = l1 * np.sin(th1)
    dy1 = -l1 * np.cos(th1)
    x1_start, y1_start = x1 - dx1, y1 - dy1
    x1_end,   y1_end   = x1 + dx1, y1 + dy1

    # Draw Link 1
    plt.plot([x1_start, x1_end], [y1_start, y1_end], color='C0')

ax = plt.gca()
ax.set_aspect('equal', 'box')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Sampled Link Configurations with Half-Length Links')
plt.show()

# Plot constraint violations
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(t_hist, C_hist, label='C(q) Violation')
plt.xlabel('Time [s]')
plt.ylabel('C(q) violation')
plt.title('Position Constraint Violation (C(q))')
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t_hist, C_dot_hist, label='C_dot(q) Violation', color='r')
plt.xlabel('Time [s]')
plt.ylabel('C_dot(q) violation')
plt.title('Velocity Constraint Violation (C_dot(q))')
plt.grid(True)

plt.tight_layout()
plt.show()

# Plot generalized coordinates q over time
fig, ax = plt.subplots()
for i in range(q_hist.shape[1]):
    ax.plot(t_hist, q_hist[:, i], label=f'q[{i}]')
ax.set_xlabel('Time [s]')
ax.set_ylabel('q components')
ax.set_title('Generalized Coordinates over Time')
ax.legend()
fig.tight_layout()
plt.show()

# Plot generalized velocities q_dot over time
fig, ax = plt.subplots()
for i in range(q_dot_hist.shape[1]):
    ax.plot(t_hist, q_dot_hist[:, i], label=f'q_dot[{i}]')
ax.set_xlabel('Time [s]')
ax.set_ylabel('q_dot components')
ax.set_title('Generalized Velocities over Time')
ax.legend()
fig.tight_layout()
plt.show()
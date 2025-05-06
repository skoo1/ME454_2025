import numpy as np
import matplotlib.pyplot as plt

# Physical constants and system parameters
m1, m2 = 0.20, 0.30    # masses [kg]
l1, l2 = 0.10, 0.15    # link half lengths [m]
I1 = m1 * l1**2 / 3    # moment of inertia link 1 [kg·m²]
I2 = m2 * l2**2 / 3    # moment of inertia link 2 [kg·m²]
g = 9.81               # gravity [m/s²]

# Initial angles (theta2 derived from closed-chain constraint)
theta1 = 1.0  # [rad]
theta2 = np.arcsin(-2 * l1 * np.sin(theta1) / (2 * l2))

# Initial positions of link ends
x1 =  l1 * np.cos(theta1)
y1 =  l1 * np.sin(theta1)
x2 = 2*l1 * np.cos(theta1) + l2 * np.cos(theta2)
y2 = 2*l1 * np.sin(theta1) + l2 * np.sin(theta2)

# Initial velocities (stationary)
q     = np.array([x1, y1, theta1, x2, y2, theta2])
q_dot = np.zeros(6)

# Define mass-inertia matrix M
M = np.diag([m1, m1, I1, m2, m2, I2])


def update_J_and_J_dot(q, q_dot):
    x1, y1, th1, x2, y2, th2 = q
    _, _, th1_dot, _, _, th2_dot = q_dot

    # Constraint Jacobian J (5 constraints × 6 generalized coordinates)
    J = np.array([
        [ 1,  0,  l1*np.sin(th1),      0,  0,               0     ],
        [ 0,  1, -l1*np.cos(th1),      0,  0,               0     ],
        [ 0,  0,  2*l1*np.sin(th1),    1,  0,  l2*np.sin(th2)     ],
        [ 0,  0, -2*l1*np.cos(th1),    0,  1, -l2*np.cos(th2)     ],
        [ 0,  0,  2*l1*np.cos(th1),    0,  0,  2*l2*np.cos(th2)   ]
    ])

    # Time derivative of J
    J_dot = np.array([
        [ 0, 0,  l1*th1_dot*np.cos(th1),     0, 0,                   0        ],
        [ 0, 0,  l1*th1_dot*np.sin(th1),     0, 0,                   0        ],
        [ 0, 0,  2*l1*th1_dot*np.cos(th1),   0, 0,  l2*th2_dot*np.cos(th2)    ],
        [ 0, 0,  2*l1*th1_dot*np.sin(th1),   0, 0,  l2*th2_dot*np.sin(th2)    ],
        [ 0, 0, -2*l1*th1_dot*np.sin(th1),   0, 0, -2*l2*th2_dot*np.sin(th2)  ]
    ])

    return J, J_dot


def update_F_ext():
    # Generalized external forces: [Fx1, Fy1, τ1, Fx2, Fy2, τ2]

    return np.array([0, -m1*g, 0, 0, -m2*g, 0])


def compute_q_ddot(q, q_dot, M):
    J, J_dot = update_J_and_J_dot(q, q_dot)
    F_ext    = update_F_ext()

    # Construct b vector
    b_top    = F_ext
    b_bottom = -np.dot(J_dot, q_dot)  # -J_dot * q_dot
    b = np.concatenate((b_top, b_bottom))

    # Construct A matrix
    A_top    = np.hstack((M, J.T))
    A_bottom = np.hstack((J, np.zeros((J.shape[0], J.shape[0]))))
    A = np.vstack((A_top, A_bottom))

    # Solve linear system for q̈ and Lagrange multipliers
    x = np.linalg.solve(A, b)
    q_ddot       = x[:6]
    lambda_prime = x[6:]

    return q_ddot, lambda_prime


# Simulation parameters
dt    = 0.001    # time step [s]
T     = 1.55     # total simulation time [s]
steps = int(T/dt)

# For storing results
q_hist     = np.zeros((steps+1, 6))
q_dot_hist = np.zeros((steps+1, 6))
t_hist     = np.linspace(0, T, steps+1)

# Record initial conditions
q_hist[0]     = q
q_dot_hist[0] = q_dot

# Main loop
for i in range(steps):
    q_ddot, _ = compute_q_ddot(q, q_dot, M)

    # Semi-implicit Euler integration
    q_dot += q_ddot * dt
    q     += q_dot   * dt

    q_hist[i+1]     = q
    q_dot_hist[i+1] = q_dot


# Plot only 100 equally-spaced configurations
num_samples = 200
indices = np.linspace(0, len(q_hist)-1, num_samples, dtype=int)

plt.figure()
for idx in indices:
    x1, y1, th1, x2, y2, th2 = q_hist[idx]

    # Link 1 endpoints (from center at x1, y1)
    dx1 = l1 * np.cos(th1)
    dy1 = l1 * np.sin(th1)
    x1_start, y1_start = x1 - dx1, y1 - dy1
    x1_end,   y1_end   = x1 + dx1, y1 + dy1

    # Link 2 endpoints (from center at x2, y2)
    dx2 = l2 * np.cos(th2)
    dy2 = l2 * np.sin(th2)
    x2_start, y2_start = x2 - dx2, y2 - dy2
    x2_end,   y2_end   = x2 + dx2, y2 + dy2

    # Draw Link 1
    plt.plot([x1_start, x1_end], [y1_start, y1_end], color='C0')

    # Draw Link 2
    plt.plot([x2_start, x2_end], [y2_start, y2_end], color='C1')

ax = plt.gca()
ax.set_aspect('equal', 'box')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Sampled Link Configurations with Half-Length Links')
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
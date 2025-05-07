import numpy as np
import matplotlib.pyplot as plt

# Physical constants and system parameters
m1, m2 = 0.20, 0.30    # masses [kg]
l1, l2 = 0.10, 0.15    # link half lengths [m]
I1 = m1 * (2*l1)**2 / 12    # moment of inertia link 1 [kg·m²]
I2 = m2 * (2*l2)**2 / 12    # moment of inertia link 2 [kg·m²]
g = 9.81               # gravity [m/s²]

# Initial angles (theta2 derived from closed-chain constraint)
theta1 = 1.0  # [rad]
theta2 = np.arcsin(-2 * l1 * np.sin(theta1) / (2 * l2))

# Initial positions of link coms
x1 =  l1 * np.cos(theta1)
y1 =  l1 * np.sin(theta1)
x2 = 2*l1 * np.cos(theta1) + l2 * np.cos(theta2)
y2 = 2*l1 * np.sin(theta1) + l2 * np.sin(theta2)

# Initial velocities (stationary)
q     = np.array([x1, y1, theta1, x2, y2, theta2])
q_dot = np.zeros(6)

# Define mass-inertia matrix M
M = np.diag([m1, m1, I1, m2, m2, I2])

# Baumgarte stabilization parameters (adjustable)
alpha = 10.0  # velocity stabilization gain
beta  = 100.0  # position stabilization gain

def update_J_and_J_dot(q, q_dot):
    x1, y1, th1, x2, y2, th2 = q
    vx1, vy1, w1, vx2, vy2, w2 = q_dot
    s1 = np.sin(th1)
    c1 = np.cos(th1)
    s2 = np.sin(th2)
    c2 = np.cos(th2)

    # Constraint Jacobian J (5 constraints × 6 generalized coordinates)
    J = np.array([
        [ 1,  0,  l1*s1,      0,  0,  0         ],
        [ 0,  1, -l1*c1,      0,  0,  0         ],
        [ 0,  0,  2*l1*s1,    1,  0,  l2*s2     ],
        [ 0,  0, -2*l1*c1,    0,  1, -l2*c2     ],
        [ 0,  0,  2*l1*c1,    0,  0,  2*l2*c2   ]
    ])

    # Time derivative of J
    J_dot = np.array([
        [ 0, 0,  l1*w1*c1,     0, 0,  0           ],
        [ 0, 0,  l1*w1*s1,     0, 0,  0           ],
        [ 0, 0,  2*l1*w1*c1,   0, 0,  l2*w2*c2    ],
        [ 0, 0,  2*l1*w1*s1,   0, 0,  l2*w2*s2    ],
        [ 0, 0, -2*l1*w1*s1,   0, 0, -2*l2*w2*s2  ]
    ])

    return J, J_dot


def update_F_ext():
    # Generalized external forces: [Fx1, Fy1, τ1, Fx2, Fy2, τ2]

    return np.array([0, -m1*g, 0, 0, -m2*g, 0])


def compute_q_ddot(q, q_dot, M, alpha, beta):
    x1, y1, th1, x2, y2, th2 = q
    c1, s1, c2, s2 = np.cos(th1), np.sin(th1), np.cos(th2), np.sin(th2)

    J, J_dot = update_J_and_J_dot(q, q_dot)
    F_ext    = update_F_ext()

    # Constraint errors (position and velocity)
    C1 = x1 - l1 * c1
    C2 = y1 - l1 * s1
    C3 = x2 - 2 * l1 * c1 - l2 * c2
    C4 = y2 - 2 * l1 * s1 - l2 * s2
    C5 = 2 * l1 * s1 + 2 * l2 * s2
    C = np.array([C1, C2, C3, C4, C5])
    C_dot = np.dot(J, q_dot)

    # Construct b vector and A matrix for Ax=b
    b_top = F_ext
    b_bottom = -J_dot @ q_dot - 2 * alpha * C_dot - beta**2 * C  # Baumgarte stabilization term
    b = np.concatenate((b_top, b_bottom))
    A = np.block([
        [M, J.T],
        [J, np.zeros((J.shape[0], J.shape[0]))]
    ])

    # Solve linear system for q̈ and Lagrange multipliers
    x = np.linalg.solve(A, b)
    q_ddot       = x[:6]
    lambda_prime = x[6:]

    return q_ddot, lambda_prime, C, C_dot


# Simulation parameters
dt    = 0.001    # time step [s]
T     = 1.55     # total simulation time [s]
steps = int(T/dt)

# For storing results
q_hist     = np.zeros((steps+1, 6))
q_dot_hist = np.zeros((steps+1, 6))
t_hist     = np.linspace(0, T, steps+1)

# For storing constraint violations
C_hist     = np.zeros(steps+1)  # C(q) violation
C_dot_hist = np.zeros(steps+1)  # C_dot(q) violation

# Record initial conditions
q_hist[0]     = q
q_dot_hist[0] = q_dot

# Main loop
for i in range(steps):
    q_ddot, _, C, C_dot = compute_q_ddot(q, q_dot, M, alpha, beta)

    # Store constraint violations
    C_hist[i+1]     = np.linalg.norm(C)
    C_dot_hist[i+1] = np.linalg.norm(C_dot)

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
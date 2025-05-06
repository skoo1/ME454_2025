import numpy as np
import matplotlib.pyplot as plt

# Physical constants and system parameters
m1, m2 = 0.20, 0.10
l1, l2 = 0.50, 0.25 # half lengths
I1 = m1 * (2*l1)**2 / 12    # moment of inertia link 1 [kg·m²]
I2 = m2 * (2*l2)**2 / 12    # moment of inertia link 2 [kg·m²]
g = 9.81
K, C = 10.0, 2.0

# Initial configuration
theta1 = np.pi / 4
theta2 = np.pi / 4
x1 = l1 * np.sin(theta1)
y1 = -l1 * np.cos(theta1)
L0 = 0.05 # nominal length of the srping
x2 =  (2*l1 + L0 + l2) * np.sin(theta2)
y2 = -(2*l1 + L0 + l2) * np.cos(theta2)
q = np.array([x1, y1, theta1, x2, y2, theta2])
q_dot = np.zeros(6)

M = np.diag([m1, m1, I1, m2, m2, I2])

def update_J_and_J_dot(q, q_dot):
    x1, y1, th1, x2, y2, th2 = q
    vx1, vy1, w1, vx2, vy2, w2 = q_dot
    c, s = np.cos(th1), np.sin(th1)
    dx, dy = x2 - x1, y2 - y1

    J = np.array([
        [1, 0, -l1 * c, 0, 0, 0],
        [0, 1, -l1 * s, 0, 0, 0],
        [-c, -s, -dx * s + dy * c, c, s, 0],
        [0, 0, 1, 0, 0, -1]
    ])

    J_dot = np.zeros((4, 6))
    J_dot[0, 2] = l1 * s * w1
    J_dot[1, 1] = 1
    J_dot[1, 2] = -l1 * c * w1
    J_dot[2, 0] = s * w1
    J_dot[2, 1] = -c * w1
    J_dot[2, 2] = -vx2*s - x2*c*w1 + vx1*s + x1*c*w1 + vy2*c - y2*s*w1 - vy1*c + y1*s*w1
    J_dot[2, 3] = -s * w1
    J_dot[2, 4] = c * w1

    return J, J_dot

def update_F_ext(q, q_dot):
    x1, y1, th1, x2, y2, th2 = q
    vx1, vy1, w1, vx2, vy2, w2 = q_dot
    c, s = np.cos(th1), np.sin(th1)

    L = np.sqrt(x2**2 + y2**2) - 2*l1 - l2 - L0
    L_dot = (x2 * vx2 + y2 * vy2) / np.sqrt(x2**2 + y2**2)
    Fsd = K * L + C * L_dot

    F_ext = np.zeros(6)
    F_ext[0] =  Fsd * s
    F_ext[1] = -Fsd * c - m1 * g
    F_ext[3] = -Fsd * s
    F_ext[4] =  Fsd * c - m2 * g

    return F_ext

def compute_q_ddot(q, q_dot):
    J, J_dot = update_J_and_J_dot(q, q_dot)
    F_ext    = update_F_ext(q, q_dot)

    # Construct b vector and A matrix for Ax=b
    b = np.concatenate((F_ext, -J_dot @ q_dot))
    A = np.block([
        [M, J.T],
        [J, np.zeros((J.shape[0], J.shape[0]))]
    ])

    x = np.linalg.solve(A, b)
    q_ddot       = x[:6]
    lambda_prime = x[6:]

    return q_ddot, lambda_prime

# Simulation parameters
dt = 0.001
T = 2.0
steps = int(T / dt)

# For storing results
q_hist     = np.zeros((steps+1, 6))
q_dot_hist = np.zeros((steps+1, 6))
t_hist     = np.linspace(0, T, steps+1)

# Record initial conditions
q_hist[0]     = q
q_dot_hist[0] = q_dot

# Simulation loop
for i in range(steps):
    q_ddot, _ = compute_q_ddot(q, q_dot)

    # Semi-implicit Euler integration
    q_dot += q_ddot * dt
    q     += q_dot   * dt

    q_hist[i+1]     = q
    q_dot_hist[i+1] = q_dot

# Visualization
num_samples = 500
indices = np.linspace(0, len(q_hist)-1, num_samples, dtype=int)
plt.figure()
for idx in indices[::10]: # every 10th
    x1, y1, th1, x2, y2, th2 = q_hist[idx]

    dx1 = l1 * np.sin(th1)
    dy1 = -l1 * np.cos(th1)
    x1_start, y1_start = x1 - dx1, y1 - dy1
    x1_end, y1_end = x1 + dx1, y1 + dy1

    dx2 = l2 * np.sin(th2)
    dy2 = -l2 * np.cos(th2)
    x2_start, y2_start = x2 - dx2, y2 - dy2
    x2_end, y2_end = x2 + dx2, y2 + dy2

    plt.plot([x1_start, x1_end], [y1_start, y1_end], color='C0')
    plt.plot([x2_start, x2_end], [y2_start, y2_end], color='C1')

ax = plt.gca()
ax.set_aspect('equal', 'box')
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Sampled Link Configurations with Prismatic Joint')
plt.show()

# Plot generalized coordinates
t = np.linspace(0, T, steps + 1)
fig, ax = plt.subplots()
for i in range(6):
    ax.plot(t, q_hist[:, i], label=f'q[{i}]')
ax.set_xlabel('Time [s]')
ax.set_ylabel('q components')
ax.set_title('Generalized Coordinates over Time')
ax.legend()
fig.tight_layout()
plt.show()

# Plot generalized velocities
fig, ax = plt.subplots()
for i in range(6):
    ax.plot(t, q_dot_hist[:, i], label=f'q_dot[{i}]')
ax.set_xlabel('Time [s]')
ax.set_ylabel('q_dot components')
ax.set_title('Generalized Velocities over Time')
ax.legend()
fig.tight_layout()
plt.show()
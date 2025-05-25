import numpy as np
import math
import matplotlib.pyplot as plt

# Quaternion utilities
def quat_mult(a, b):
    w0,x0,y0,z0 = a
    w1,x1,y1,z1 = b
    return np.array([
        w0*w1 - x0*x1 - y0*y1 - z0*z1,
        w0*x1 + x0*w1 + y0*z1 - z0*y1,
        w0*y1 - x0*z1 + y0*w1 + z0*x1,
        w0*z1 + x0*y1 - y0*x1 + z0*w1
    ])

def quat_conj(a):
    return np.array([a[0], -a[1], -a[2], -a[3]])

def quat_rotate(q, v):
    v_q = np.hstack(([0.0], v))
    return quat_mult(quat_mult(q, v_q), quat_conj(q))[1:]

def quat_to_matrix(q):
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),   2*(y*z + x*w), 1-2*(x*x+y*y)]
    ])

def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])

def norm_q(q):
    return q/np.linalg.norm(q)

def compute_J(R, s_local):
    """
    J = [E3  -[R s_local]_×], shape (3,6)
    """
    Rs = R.dot(s_local)
    J_r = np.eye(3)
    J_w = -skew(Rs)
    # concatenate (3×3) and (3×3) laterally (3×6)
    return np.hstack([J_r, J_w])

def compute_J_dot(omega, R, s_local):
    """
    J_dot = [0  -[ω×(R s_local)]_×], shape (3,6)
    """
    Rs = R @ s_local
    omega_cross_Rs = np.cross(omega, Rs)
    J_r_dot = np.zeros((3,3))
    J_w_dot = -skew(omega_cross_Rs)
    return np.hstack([J_r_dot, J_w_dot])


# === Simulation Parameters ===
dt = 0.001          # Time step [s]
steps = 500        # Number of simulation steps
g = 9.81           # Gravity [m/s^2]

# === Toy Top Geometry & Inertia ===
m = 0.1            # mass [kg]
height = 0.10      # full height [m]
r_cm_to_tip = height / 2  # COM-to-tip distance

# Local position vector from COM to pivot (body frame)
s_local = np.array([0.0, 0.0, -r_cm_to_tip])

# Body-frame inertia tensor (cone approximation)
I_body = np.diag([5e-5, 5e-5, 1e-4])     # Izz = 2 Ixx

# === Initial State ===
# Initial quaternion (3° tilt about x)
tilt_rad = np.radians(3.0)
q = norm_q(np.array([np.cos(tilt_rad/2), np.sin(tilt_rad/2), 0.0, 0.0]))  # [w, x, y, z]

spin_rpm    = 2500.0
omega_z_body = 2*math.pi*spin_rpm/60
R = quat_to_matrix(q)

# === calculate good initial velocity for simulation
d      = np.linalg.norm(R @ s_local)        # ‖rW‖
Izz    = I_body[2, 2]
precession_rate = m*9.81*d / (Izz*omega_z_body) # precession rate   Ω = mgd/(Iω)

kW     = np.array([0, 0, 1])           # +z (up)
eW     = R[:, 2]                       # body-z in world
omega_world = precession_rate*kW + omega_z_body*eW
omega_body  = R.T @ omega_world        # store body-frame

v_lin_world = np.cross(omega_world, R @ s_local)
x = - R @ s_local                      # initial COM position


# === semi-implicit Euler forward dynamics loop ===
xs, ys, zs = [], [], []

for _ in range(800):
    R = quat_to_matrix(q)
    omega_world = R @ omega_body

    # compute mass matrix
    I_world = R @ I_body @ R.T
    M = np.block([
        [m * np.eye(3),            np.zeros((3,3))],
        [np.zeros((3,3)),          I_world        ]
    ])  # shape (6,6)

    # compute jacobian
    J = compute_J(R, s_local)  # shape (3,6)

    A = np.block([[M, -J.T],
                  [J, np.zeros((3,3))]])

    Fext= np.hstack([np.array([0.0, 0.0, -m * g]), np.zeros(3)])

    C = np.hstack([np.zeros(3), np.cross(omega_world, I_world @ omega_world)])

    J_dot = compute_J_dot(omega_world, R, s_local)

    Cpos   = x + R @ s_local
    beta, gamma = 50.0, 60.0           # Baumgarte gains
    rhsAcc = (- J_dot @ np.hstack([v_lin_world, omega_world])
              - 2*beta * J @ np.hstack([v_lin_world, omega_world])
              - gamma * gamma * Cpos)
    
    rhs = np.hstack([Fext - C, rhsAcc])

    sol = np.linalg.solve(A, rhs)
    accel_lin_world, accel_rot_world = sol[:3], sol[3:6]

    # semi-implicit integrate
    v_lin_world  += dt * accel_lin_world;
    x  += dt * v_lin_world

    # quaternion update (dq ⊗ q)  –– dq is world-frame
    omega_body += dt * (R.T @ accel_rot_world)
    omega_world_new = R @ omega_body
    n = np.linalg.norm(omega_world_new)
    dq = (np.array([1,0,0,0])
          if n < 1e-10 else
          np.hstack([math.cos(0.5*n*dt),
                     math.sin(0.5*n*dt)*omega_world_new/n]))
    q = norm_q(quat_mult(dq, q))

    xs.append(x[0]);  ys.append(x[1]);  zs.append(x[2])


# plot the result - Trajectory of the COM
plt.figure(figsize=(6,6))
plt.plot(xs, ys)
plt.title('COM trajectory – near-perfect circle')
plt.xlabel('x (m)'); plt.ylabel('y (m)')
plt.gca().set_aspect('equal', 'box'); plt.grid(True)
plt.show()

plt.figure(figsize=(6,6))
plt.plot(xs, zs)
plt.title('COM trajectory – near-perfect circle')
plt.xlabel('x (m)'); plt.ylabel('z (m)')
plt.gca().set_aspect('equal', 'box'); plt.grid(True)
plt.show()
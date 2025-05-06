import numpy as np
import matplotlib.pyplot as plt

# ========= USER OPTIONS FOR TANGENTIAL FORCE ========================
TAN_MODEL = 1     # 1 = always-integrate  |  2 = stick-only integrate
# ====================================================================

# -------------------------------------------------
# CONFIG
# -------------------------------------------------
g      = 9.81          # gravity (m/s²)
R      = 0.02          # ball radius (m)
m      = 0.02          # ball mass (kg)
mu     = 0.25          # Coulomb friction coefficient
k_n    = 5.0e4         # normal contact stiffness
c_n    = 2.0           # normal contact damping 
k_t    = 3.0e4         # tangential contact stiffness
c_t    = 5.0           # tangential contact damping

x0, z0 = 0.0, 1.0      # initial position (m)
v_x0   = 0.5           # initial horizontal velocity  (+x forward)
v_z0   = 0.0           # initial vertical velocity    (downward)
omega0 = -50.0         # initial spin (rad/s, − = back‑spin)

dt     = 1e-3          # simulation time step (s)
t_end  = 3.0           # total simulated time (s)
# -------------------------------------------------

I = 0.4 * m * R**2     # moment of inertia (solid sphere)

# state variables
x, z = x0, z0
v_x, v_z, omega = v_x0, v_z0, omega0
shear_disp = 0.0

ts, xs, zs, vxs = [], [], [], []
t = 0.0

# -------------- main loop --------------
while t <= t_end:
    ts.append(t)
    xs.append(x)
    zs.append(z)

    # force and torque
    f_x = 0.0
    f_z = -m * g
    tau = 0.0

    pen = R - z
    if pen > 0.0:
        # --- normal Kelvin model ---
        f_n = k_n * pen + c_n * (-v_z)
        f_n = max(f_n, 0.0)
        f_z += f_n

        # --- tangential stick / slip ---
        u = v_x - R * omega

        # trial stick force
        if TAN_MODEL == 1:
            shear_disp += u * dt              # always integrate
        f_el, f_d = -k_t * shear_disp, -c_t * u
        f_t_trial = f_el + f_d
        f_t_max   = mu * f_n

        if abs(f_t_trial) <= f_t_max:         # STICK
            f_t = f_t_trial
            if TAN_MODEL == 2:                # stick-only integrate
                shear_disp += u * dt
        else:                                 # SLIP
            f_t = -np.sign(u) * f_t_max
            shear_disp = -f_t / k_t           # clamp spring

        f_x += f_t
        tau  = -f_t * R
    else:
        shear_disp = 0.0

    # --- semi-implicit update ---
    a_x, a_z = f_x/m, f_z/m
    alpha = tau/I
    v_x += a_x*dt
    v_z += a_z*dt
    omega += alpha*dt
    vxs.append(v_x)

    x += v_x*dt
    z += v_z*dt
    t += dt
# =================================================================

# ---------- plots ----------
plt.figure(figsize=(6,4))
plt.plot(xs, zs, lw=2)
plt.axhline(R, color='k')
plt.gca().set_aspect('equal','box')
plt.xlabel('x (m)'); plt.ylabel('z (m)')
plt.title(f'Compliance model  (TAN_MODEL={TAN_MODEL})')
plt.tight_layout(); plt.savefig('trajectory.png', dpi=150)

plt.figure(figsize=(6,3))
plt.plot(ts, vxs, lw=1)
plt.xlabel('time (s)'); plt.ylabel('v_x (m/s)')
plt.title('Horizontal speed (logged)')
plt.tight_layout(); plt.savefig('vx_t.png', dpi=150)

print('Finished.  Files: trajectory.png, vx_t.png   |   TAN_MODEL =', TAN_MODEL)

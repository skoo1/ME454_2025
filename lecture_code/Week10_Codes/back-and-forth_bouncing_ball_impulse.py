import numpy as np
import matplotlib.pyplot as plt

# -------------------------------------------------
# CONFIG
# -------------------------------------------------
g      = 9.81          # gravity (m/s²)
R      = 0.02          # ball radius (m)
m      = 0.02          # ball mass (kg)
mu     = 0.25          # Coulomb friction coefficient
e      = 0.95          # normal coefficient of restitution (0–1)
e_t    = 0.6           # tangential restitution (0=no rebound, <1)

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

ts, xs, zs, vxs, vzs, omegas = [], [], [], [], [], []
t = 0.0

# -------------- main loop --------------
while t <= t_end:
    ts.append(t)
    xs.append(x)
    zs.append(z)

    # free flight
    v_z -= g * dt
    x   += v_x * dt
    z   += v_z * dt
    J_n = 0.0
    J_t = 0.0

    # ground collision (plane at z = R)
    if z <= R and v_z < 0.0:
        z = R

        # ------ normal impulse ------
        v_z_pre = v_z
        v_z = -e * v_z_pre # v_z_post
        J_n = (v_z - v_z_pre) * m

        # ------ tangential impulse with restitution ------
        u_pre   = v_x - R * omega            # slip before impact
        u_post  = -e_t * u_pre               # desired slip after impact

        J_t_star = (u_post - u_pre) * (2.0 * m / 7.0)  # needed impulse
        J_t_max  = mu * J_n                            # Coulomb limit
        # J_t : tangential impulse
        # del_v_x = J_t/m
        # del_omega = -J_t*R/I
        # del_u = u_post - u_pre = del_v_x - R*del_omega = 7*J_t/(2*m)
        # This J_t is the needed impulse J_t_impulse

        # stick / slip decision
        if abs(J_t_star) <= J_t_max:
            J_t = J_t_star                    # sticking (or partial)
        else:
            J_t = -np.sign(u_pre) * J_t_max   # sliding, saturated

    # update velocities
    v_x   += J_t / m
    omega -= J_t * R / I
    vxs.append(v_x)
    vzs.append(v_z)
    omegas.append(omega/10)

    t += dt
# ----------------------------------------

# -------------- plot ---------------------
plt.figure(figsize=(6,4))
plt.plot(xs, zs, lw=2)
plt.axhline(R, color='k', lw=1)
plt.gca().set_aspect('equal', 'box')
plt.xlabel('x (m)')
plt.ylabel('z (m)')
plt.title('Back‑Spinning Ball with tangential restitution')
plt.tight_layout()
plt.savefig('trajectory.png', dpi=150)
print('Simulation complete. Plot saved as trajectory.png')

# ---------- plots ----------
plt.figure(figsize=(6,4))
plt.plot(ts, xs, lw=2, label='x position (m)')
plt.plot(ts, zs, lw=2, label='z position (m)')
plt.axhline(R, color='k', label='Ground level')
plt.gca().set_aspect('equal','box')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.title('Position vs Time')
plt.legend()
plt.tight_layout()
plt.savefig('trajectory.png', dpi=150)

plt.figure(figsize=(6,3))
plt.plot(ts, vxs, lw=1, label='v_x (m/s)')
plt.plot(ts, vzs, lw=1, label='v_z (m/s)')
plt.plot(ts, omegas, lw=1, label='omega (rad/s) / 10')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity vs Time')
plt.legend()
plt.tight_layout()
plt.savefig('velocity.png', dpi=150)

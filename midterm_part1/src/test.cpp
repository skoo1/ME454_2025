// Test code for KAIST ME454 Dynamics System Programming course
// 2025 April, KAIST MSKBioDyn

// This file is not evaluated. Only the simulation file (simulation.cpp) will be evaluated for Part 1.

#include "singlebodysim.hpp"

int main()
{
    
    // Initial conditions
    double dt = 0.001; // timestep (s)
    double mass = 0.310; // mass (kg)
    Mat33 inertia(0.01404, 0.00263, 0.01662); // inertia (kg m^2)

    Vec3 pos_init(0.0, 0.0, 1.0); // initial position (m)
    Quaternion rot_init(1.0, 0.0, 0.0, 0.0); // initial rotation / orientation (quaternion)
    Vec3 lin_vel_init(0.0, 0.0, 10.0); // initial linear velocity (m/s)
    Vec3 ang_vel_init(10.0, 0.001, 0.0); // initial angular velocity (rad/s)

    Vec3 gravity(0.0, 0.0, -9.81);
    Vec3 force = gravity * mass; // gravitational force
    Vec3 torque(0.0, 0.0, 0.0); // no torque

    // Simulation settings
    SingleBodySim sb_sim;
    sb_sim.set_dt(dt);
    sb_sim.set_inertia(mass, inertia);
    sb_sim.set_pose(pos_init, rot_init);
    sb_sim.set_vel_wf(lin_vel_init, ang_vel_init);
    
    std::cout << "Simulation Started" << std::endl;

    // Simulation loop
    int max_step = 2001;
    for (int step = 0; step < max_step; step++)
    {
        // Print the states every 100 steps
        if (step % 100 == 0) sb_sim.print_state();
        sb_sim.integrate(force, torque);
    }
    
    std::cout << "Order Code : " << SingleBodySim::order_code_ << std::endl;

    std::cout << "Simulation Finished" << std::endl;
    return 0;

}
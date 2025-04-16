// Single body simulator source code for KAIST ME454 Dynamics System Programming course
// April 2025, KAIST MSKBioDyn Lab.

// Only this file will be evaluated for Part 1.
// Implemented by [YOUR NAME], [YOUR STUDENT ID]

#include "singlebodysim.hpp"

// Constructor of the SingleBodySim class
SingleBodySim::SingleBodySim() 
{
}

void SingleBodySim::set_dt(double dt) {dt_ = dt;};
void SingleBodySim::set_inertia(double mass, Mat33 inertia) {mass_ = mass; inertia_bf_ = inertia;}
void SingleBodySim::set_pose(Vec3 pos, Quaternion rot) {pos_wf_ = pos; rot_b2w_ = rot;}
void SingleBodySim::set_vel_wf(Vec3 lin_vel_wf, Vec3 ang_vel_wf)
{
    lin_vel_wf_ = lin_vel_wf;
    ang_vel_bf_ = transform_w2b(ang_vel_wf);
}

double SingleBodySim::get_time() {return time_;};
void SingleBodySim::get_pose(Vec3& pos, Quaternion& rot) {pos = pos_wf_; rot = rot_b2w_;}
void SingleBodySim::get_vel_wf(Vec3& lin_vel, Vec3& ang_vel)
{
    lin_vel = lin_vel_wf_;
    ang_vel = transform_b2w(ang_vel_bf_);
}

void SingleBodySim::print_state()
{
    std::cout << "Time : \t" << time_ << std::endl;
    std::cout << "Position : \t\t\t";
    pos_wf_.display();
    std::cout << "Rotation : \t\t\t";
    rot_b2w_.display();
    std::cout << "Linear Velocity : \t\t";
    lin_vel_wf_.display();
    std::cout << "Angular Velocity (Body) : \t";
    ang_vel_bf_.display();
    std::cout << "Angular Velocity (World) : \t";
    transform_b2w(ang_vel_bf_).display();
    std::cout << std::endl;
}

// Problem 1. Complete the SingleBodySim class (total 60 pts)
// Total 60 pts: (4 + 4 + 6 + 8 + 6 + 4 + 10 + 4 + 4 + 6 = 56 pts partial credit) + (4 pts if the integrate function is perfect)
// Your implementation starts here

// Class member functions for vector transformation

// Problem 1-1. Transform a vector [vector] from world to body coordinates (4 pts)
Vec3 SingleBodySim::transform_w2b(Vec3 vector)
{

    return vector;
}

// Problem 1-2. Transform a vector [vector] from body to world coordinates (4 pts)
Vec3 SingleBodySim::transform_b2w(Vec3 vector)
{

    return vector;
}

// TA TODO: make print_order function to evaluate Problem 1-3.

void SingleBodySim::integrate(Vec3 force_wf, Vec3 torque_wf)
{
    Vec3 axis;
    double angle;
    Quaternion d_rot;
    Vec3 d_ang_vel;
    Vec3 torque_bf = transform_w2b(torque_wf);

    order_code_ = 0;

    // Problem 1-3. This part is implemented with explicit Euler integration.
    //              Change the order of the subfunctions to apply semi-implicit Euler integration. (6 pts)
    //              Do not change the function parameters!
    
    calc_axis_from_wdt(dt_, ang_vel_bf_, axis, angle);
    convert_axis_to_quat(axis, angle, d_rot);
    update_rot(d_rot, rot_b2w_);

    calc_newton_euler(inertia_bf_, torque_bf, ang_vel_bf_, dt_, d_ang_vel); // equation of motion in the body frame
    update_ang_vel(d_ang_vel, ang_vel_bf_);

    update_pos(lin_vel_wf_, dt_, pos_wf_);

    update_lin_vel(mass_, force_wf, dt_, lin_vel_wf_); // equation of motion in the world frame

    time_ = time_ + dt_;
}

int SingleBodySim::order_code_ = 0;

// Sub-functions for the integrate function

// Problem 1-4. Calculate differential rotation as [angle] and [axis] from angular velocity [ang_vel] and timestep [dt] (8 pts)
void calc_axis_from_wdt(double dt, Vec3 ang_vel, Vec3& axis, double& angle)
{

}

// Problem 1-5. Convert differential rotation [d_rot] from [angle] and [axis] to quaternion (6 pts)
void convert_axis_to_quat(Vec3 axis, double angle, Quaternion& d_rot)
{

}

// Problem 1-6. Update rotation [rot] using the differential rotation [d_rot] (4 pts)
void update_rot(Quaternion d_rot, Quaternion& rot)
{

    SingleBodySim::order_code_ &= 13; // Do not modify this line!
}

// Problem 1-7. Calculate differential angular velocity [d_ang_vel] with the Newton-Euler equation given [inertia], [torque], and [ang_vel] (10 pts)
void calc_newton_euler(Mat33 inertia, Vec3 torque, Vec3 ang_vel, double dt, Vec3& d_ang_vel)
{

}

// Problem 1-8. Update angular velocity [ang_vel] using the differential angular velocity [d_ang_vel] (4 pts)
void update_ang_vel(Vec3 d_ang_vel, Vec3& ang_vel)
{

    SingleBodySim::order_code_ |= 3; // Do not modify this line!
}

// Problem 1-9. Update position [pos] using linear velocity [lin_vel] (4 pts)
void update_pos(Vec3 lin_vel, double dt, Vec3& pos)
{

    SingleBodySim::order_code_ &= 7; // Do not modify this line!
}

// Problem 1-10. Update linear velocity [lin_vel] with Newton equation given [mass], and [force] on com (6 pts)
void update_lin_vel(double mass, Vec3 force, double dt, Vec3& lin_vel)
{

    SingleBodySim::order_code_ |= 12; // Do not modify this line!
}

// Single body simulator source code for KAIST ME454 Dynamics System Programming course  
// April 2025, KAIST MSKBioDyn Lab.

// Do not modify the header file! Only the source file (simulation.cpp) will be evaluated for Part 1.

#ifndef _SIMULATION_H_
#define _SIMULATION_H_
#include "mymat.hpp"
#include "myquaternion.hpp"
#include <cmath>

class SingleBodySim
{
    public:

        SingleBodySim(); // constructor

        void set_dt(double dt); // set simulation timestep
        void set_inertia(double mass, Mat33 inertia); // set mass and inertia of the rigid body
        void set_pose(Vec3 pos, Quaternion rot); // set position and orientation of the rigid body
        void set_vel_wf(Vec3 lin_vel, Vec3 ang_vel); // set linear and angular velocities of the rigid body (in world frame)

        double get_time();
        void get_pose(Vec3& pos, Quaternion& rot); // get position and orientation of the rigid body
        void get_vel_wf(Vec3& lin_vel, Vec3& ang_vel); // get linear and angular velocities of the rigid body (in world frame)

        void print_state();

        void integrate(Vec3 force_wf, Vec3 torque_wf); // integrate the pose and velocity given external force
        
        static int order_code_; // for evaluation
        
    private:
        // NOTE: Underscore (_) after the variable name denotes a member variable.

        double dt_, time_; // Simulation timestep and elapsed time

        // Kinematics
        // wf: vector coordinates in the [world frame]
        // bf: vector coordinates in the [body frame]
        Vec3 pos_wf_, lin_vel_wf_, ang_vel_bf_;

        // b2w: transform from body frame coordinates to world frame coordinates
        // vec_wf = R_b2w * vec_bf << here R_b2w is a rotation matrix, but we use quaternion.
        Quaternion rot_b2w_;

        // Inertial properties
        double mass_; // mass is constant
        Mat33 inertia_bf_; // inertia is constant in the body frame
        
        // Member functions for vector transformation
        Vec3 transform_w2b(Vec3 vector);
        Vec3 transform_b2w(Vec3 vector);
};

// Subfunctions for integrate function in SingleBodySim, to be tested individually

// Update rotation
void calc_axis_from_wdt(double dt, Vec3 ang_vel, Vec3& axis, double& angle);
void convert_axis_to_quat(Vec3 axis, double angle, Quaternion& d_rot);
void update_rot(Quaternion d_rot, Quaternion &rot);

// Update angular velocity
void calc_newton_euler(Mat33 inertia, Vec3 torque, Vec3 ang_vel, double dt, Vec3& d_ang_vel);
void update_ang_vel(Vec3 d_ang_vel, Vec3& ang_vel);

// Update position and velocity
void update_pos(Vec3 lin_vel, double dt, Vec3 &pos);
void update_lin_vel(double mass, Vec3 force, double dt, Vec3 &lin_vel);

#endif // _SIMULATION_H_

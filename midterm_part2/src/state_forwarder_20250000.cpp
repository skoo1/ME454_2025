// ROS node subscriber-publisher source code for KAIST ME454 Dynamics System Programming course
// 2025 April, KAIST MSKBioDyn Lab.

// Only this file will be evaluated for Part 2.
// Implemented by [YOUR NAME], [YOUR STUDENT ID]

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;

// Problem 2. Implement the StateForwarder node (total 40 pts)
// Total 40 pts: (20 + 10 + 10 pts partial credit)

// Problem 2-1. Subscribe to the model states from Gazebo and print the estimated time in the following format:
//              "Estimated Sim Time: 0.000000 s"    (20 pts)

// Problem 2-2. Publish the racket pose so that the state_printer node can print it (10 pts).

// Problem 2-3. Publish the racket velocity so that the state_printer node can print it (10 pts).

class StateForwarder : public rclcpp::Node
{
public:
StateForwarder()
    : Node("state_forwarder_20235136") // Your student ID here
    {
        // TODO: Add a member subscription and publishers.

    }
private:

    // TODO: Add a callback function
    // Note: Do not use keyword 'const' for the function so that it can modify the member variable.


    // TODO: Declare member variables here.
    
    int count_ = 0; // estimated time = count_ * message period (in sim time)
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateForwarder>());
    rclcpp::shutdown();

    return 0;
}
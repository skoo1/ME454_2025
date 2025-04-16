// ROS node subscriber source code for KAIST ME454 Dynamics System Programming course
// 2025 April, KAIST MSKBioDyn Lab.

// This file is not evaluated. Only the state forwarder node (state_forwarder_cpp) will be evaluated for Part 2.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;

class StatePrinter : public rclcpp::Node
{
public:
StatePrinter()
    : Node("state_printer")
    {
        subscription_pose_ = this->create_subscription<geometry_msgs::msg::Pose> (
                "racket/pose", 10,
                std::bind(&StatePrinter::topic_pose_callback, this, _1)
                );
        
        subscription_vel_ = this->create_subscription<geometry_msgs::msg::Twist> (
                "racket/vel", 10,
                std::bind(&StatePrinter::topic_vel_callback, this, _1)
                );
    }
private:

    void topic_pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Position :  %f \t%f \t%f ", msg->position.x, msg->position.y, msg->position.z);
        RCLCPP_INFO(this->get_logger(), "Orientation :  %f \t%f \t%f \t%f", msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    }
    
    void topic_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Linear Velocity :  %f \t%f \t%f", msg->linear.x, msg->linear.y, msg->linear.z);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity :  %f \t%f \t%f\n", msg->angular.x, msg->angular.y, msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_vel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatePrinter>());
    rclcpp::shutdown();

    return 0;
}
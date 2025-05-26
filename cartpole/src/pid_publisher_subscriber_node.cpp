#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <math.h>

const double Pi = 3.1415926535897931;
const double HalfPi = Pi / 2;

class PIDPublisherSubscriberNode : public rclcpp::Node
{
public:
    
    // Class variables for cartpole control; these should to be initialized in the constructor.
    double x, theta; //variables for cartpole configuration in current time step.
    double dt; // Simulation time step
    double error_theta_old, error_x_old; // Errors from the previous time step, used for derivative calculation.
    // TODO END
    PIDPublisherSubscriberNode()
        : Node("pid_publisher_subscriber_node")
    {
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cartpole/joint_states", 10,
            std::bind(&PIDPublisherSubscriberNode::pole_joint_angle_callback, this, std::placeholders::_1));

        force_input_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller1/commands", 10);

        // TODO: Initialize variables with specific values (do not modify dt)
        dt = 0.001; // it depends on the update rate of controller
    
        // TODO End

        RCLCPP_INFO(this->get_logger(), "PIDPublisherSubscriberNode has been started.");
    }

private:

    void pole_joint_angle_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
        //Find joint indices of each joint: pole_joint and cart_rail_joint.
        int pole_joint_index = -1;
        int rail_cart_joint_index = -1;


        for (size_t i = 0; i < msg->name.size(); ++i){
            if (msg->name[i] == "pole_joint"){
                pole_joint_index = i;
                break;
            }
        }if (pole_joint_index == -1){
            RCLCPP_ERROR(this->get_logger(), "pole_joint not found in gazebo link_states");
            return;
        }

        for (size_t i = 0; i < msg->name.size(); ++i){
            if (msg->name[i] == "cart_rail_joint"){
                rail_cart_joint_index = i;
                break;
            }
        }if (rail_cart_joint_index == -1){
            RCLCPP_ERROR(this->get_logger(), "cart_rail_joint not found in gazebo link_states");
            return;
        }

        // Assign values from the received msg.
        theta = msg->position[pole_joint_index];
        // omega = msg->velocity[pole_joint_index];

        x = msg->position[rail_cart_joint_index];
        // v = msg->velocity[rail_cart_joint_index];

        // Publish calculated force input to the controller
        std_msgs::msg::Float64MultiArray force_input;
        force_input.data.push_back(get_inputForce_pid_controller()); // Repeat this line for additional effort controllers.
        force_input_publisher_->publish(force_input);

        return;
    }

    double get_inputForce_pid_controller()
    {
        // Implement the PID controller logic in this function.
        // Uses class member variables: cart position (x), pole angle (theta), and previous errors (error_theta_old, error_x_old)
        // Output: Control force for the controller.

        // Variable initialization.
        double force_input = 0;

        double P_gain_theta = -316.23;
        double I_gain_theta = 0.2;
        double D_gain_theta = -68.454;

        double P_gain_x = -51.675;
        double I_gain_x = 0.1;
        double D_gain_x = -39.677;

        // If gains for the position error term are not set, the cart will slide along the rail (example case).
        // double P_gain_x = 0; 
        // double I_gain_x = 0;
        // double D_gain_x = 0;

        double target_angle = 0.0;
        double target_x = 0.0;

        // TODO: Impliment the force input calculation using PID control here.

        // TODO End

        return force_input;
    }
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr force_input_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPublisherSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}

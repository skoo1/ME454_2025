#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/link_states.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <math.h>

const double Pi = 3.1415926535897931;
const double HalfPi = Pi / 2;

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class PIDPublisherSubscriberNode : public rclcpp::Node
{
public:
    double x, v, theta, omega, x_old, theta_old, dt, error_theta_old, error_x_old;
    PIDPublisherSubscriberNode()
        : Node("pid_publisher_subscriber_node")
    {
        joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cartpole/joint_states", 10,
            std::bind(&PIDPublisherSubscriberNode::pole_joint_angle_callback, this, std::placeholders::_1));

        force_input_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/effort_controller1/commands", 10);

        dt = 0.001;
        x_old = x = 0;
        v = 0;
        theta_old = theta = 0;
        omega = 0;
        error_theta_old = error_x_old = 0;
        RCLCPP_INFO(this->get_logger(), "PIDPublisherSubscriberNode has been started.");
    }

private:

    void pole_joint_angle_callback(const sensor_msgs::msg::JointState::SharedPtr msg){

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
            RCLCPP_ERROR(this->get_logger(), "pole_joint not found in gazebo link_states");
            return;
        }

        theta = msg->position[pole_joint_index];
        x = msg->position[rail_cart_joint_index];

        std_msgs::msg::Float64MultiArray force_input;
        force_input.data.push_back(get_inputForce_pid_controller());        
        force_input_publisher_->publish(force_input);
        x_old = x;
        theta_old = theta;
        return;
    }

    double get_inputForce_pid_controller()
    {
        double force_input = 0;

        double P_gain_theta = -316.23;
        double I_gain_theta = 0.2;
        double D_gain_theta = -68.454;

        double P_gain_x = -51.675;
        double I_gain_x = 0.1;
        double D_gain_x = -39.677;

        double target_angle = 0.0;
        double target_x = 0.0;

        double error_theta = target_angle - theta;
        double error_x = target_x - x;

        I_error_theta += (target_angle - theta) * dt;
        I_error_x     += (target_angle - x    ) * dt;

        force_input += P_gain_theta * error_theta;
        force_input += I_gain_theta * I_error_theta;
        force_input += D_gain_theta * (error_theta - error_theta_old) / dt ;
        
        force_input += P_gain_x     * error_x     ;
        force_input += I_gain_x     * I_error_x   ;
        force_input += D_gain_x     * (error_x     - error_x_old    ) / dt ;

        error_theta_old = error_theta;
        error_x_old = error_x;

        //return force_input;
        return 0;
    }
    double I_error_theta=0.0;
    double I_error_x=0.0;
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

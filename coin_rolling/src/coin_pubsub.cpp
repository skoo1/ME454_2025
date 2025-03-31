#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"



using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node
{
public:
    MinimalPubSub()
    : Node("coin_pubsub")
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&MinimalPubSub::topic_callback, this, _1)
                );

        publisher_ = this->create_publisher<std_msgs::msg::Float32>("coin/coin_angvel_y",10);
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        double wy = msg->twist[1].angular.y;
        auto message = std_msgs::msg::Float32();
        message.data = wy;
        publisher_->publish(message);
    }
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;

    double mass_ = 10.0;
    double ixx_ = 0.7;
    double iyy_ = 0.7;
    double izz_ = 1.25;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPubSub>());
    rclcpp::shutdown();

    return 0;
}
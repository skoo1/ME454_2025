#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "coin_rolling_msg/msg/coin_rolling.hpp"
#include "mymat.hpp"

using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node
{
public:
    MinimalPubSub()
    : Node("inclined_coin_pubsub")
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&MinimalPubSub::topic_callback, this, _1)
                );

        publisher_ = this->create_publisher<coin_rolling_msg::msg::CoinRolling>("coin/inclined_coin",10);
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {

        auto message_pose = msg->pose[1];
        auto message_twist = msg->twist[1];

        geometry_msgs::msg::Vector3 message_momentum;

        //////////////// TODO ////////////////
        // Calculate the angular momentum vector of the rolling coin on the world frame
        // and publiseh the answer.

        message_momentum.x = 0.0;
        message_momentum.y = 0.0;
        message_momentum.z = 0.0;

        //////////////// TODO End ////////////////

        auto message = coin_rolling_msg::msg::CoinRolling();
        message.angular_momentum = message_momentum;
        message.pose = message_pose;
        message.twist = message_twist;
        publisher_->publish(message);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
    rclcpp::Publisher<coin_rolling_msg::msg::CoinRolling>::SharedPtr publisher_;

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

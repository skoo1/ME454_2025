#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;

geometry_msgs::msg::Vector3 vector_cross(geometry_msgs::msg::Vector3 a, geometry_msgs::msg::Vector3 b)
{
    double ax = a.x;
    double ay = a.y;
    double az = a.z;
    double bx = b.x;
    double by = b.y;
    double bz = b.z;

    double cx = (ay*bz) - az*by;
    double cy = (az*bx) - ax*bz;
    double cz = (ax*by) - ay*bx;

    auto c = geometry_msgs::msg::Vector3();
    c.x = cx;
    c.y = cy;
    c.z = cz;

    return c;
}


class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("coin_subscriber")
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&MinimalSubscriber::topic_callback, this, _1)
                );
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        int current_model_idx = 1;
        RCLCPP_INFO(this->get_logger(), "Current model name is %s.", msg->name[current_model_idx].c_str());

        double wy = msg->twist[current_model_idx].angular.y;
        double vx = msg->twist[current_model_idx].linear.x;

        

        RCLCPP_INFO(this->get_logger(), "Linear velocity :  %f ", vx);
        RCLCPP_INFO(this->get_logger(), "Angular velocity :  %f ", wy);

    }
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;

    bool knowIndexOfModels_;
    int modelIndex_;
    std::string modelName_;

    double mass_ = 10.0;
    double ixx_ = 0.7;
    double iyy_ = 0.7;
    double izz_ = 1.25;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();

    return 0;
}
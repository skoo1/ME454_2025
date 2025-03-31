#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "coin_rolling_msg/msg/coin_rolling.hpp"
#include "math.h"
#include <functional>


using std::placeholders::_1;

std::vector<double> i_world_;
std::vector<double> angvel_world_= std::vector<double>(3,0.0);
std::vector<double> linvel_world_= std::vector<double>(3,0.0);
std::vector<double> position_world_ = std::vector<double>(3,0.0);

std::vector<double> eigen_matrix(){
    std::vector<double> mat;
    mat.push_back(1.0);
    mat.push_back(0.0);
    mat.push_back(0.0);
    mat.push_back(0.0);
    mat.push_back(1.0);
    mat.push_back(0.0);
    mat.push_back(0.0);
    mat.push_back(0.0);
    mat.push_back(1.0);
    return mat;
}

std::vector<double>std_vec_const_mult(std::vector<double> mat, double a){
    std::transform(mat.begin(), mat.end(), mat.begin(), std::bind(std::multiplies<double>(), _1, a));
    return mat;
}

std::vector<double>mat_addition(std::vector<double> mat1, std::vector<double> mat2){
    double mat1_33 = mat1.back();
    mat1.pop_back();
    double mat1_32 = mat1.back();
    mat1.pop_back();
    double mat1_31 = mat1.back();
    mat1.pop_back();
    double mat1_23 = mat1.back();
    mat1.pop_back();

    double mat1_22 = mat1.back();
    mat1.pop_back();

    double mat1_21 = mat1.back();
    mat1.pop_back();

    double mat1_13 = mat1.back();
    mat1.pop_back();

    double mat1_12 = mat1.back();
    mat1.pop_back();

    double mat1_11 = mat1.back();
    mat1.pop_back();    

    double mat2_33 = mat2.back();
    mat2.pop_back();

    double mat2_32 = mat2.back();
    mat2.pop_back();

    double mat2_31 = mat2.back();
    mat2.pop_back();

    double mat2_23 = mat2.back();
    mat2.pop_back();

    double mat2_22 = mat2.back();
    mat2.pop_back();

    double mat2_21 = mat2.back();
    mat2.pop_back();

    double mat2_13 = mat2.back();
    mat2.pop_back();

    double mat2_12 = mat2.back();
    mat2.pop_back();

    double mat2_11 = mat2.back();
    mat2.pop_back();

    std::vector<double> mat;
    mat.push_back(mat1_11 + mat2_11);
    mat.push_back(mat1_12 + mat2_12);
    mat.push_back(mat1_13 + mat2_13);
    mat.push_back(mat1_21 + mat2_21);
    mat.push_back(mat1_22 + mat2_22);
    mat.push_back(mat1_23 + mat2_23);
    mat.push_back(mat1_31 + mat2_31);
    mat.push_back(mat1_32 + mat2_32);
    mat.push_back(mat1_33 + mat2_33);
    return mat;
}

std::vector<double>mat_substraction(std::vector<double> mat1, std::vector<double> mat2){
    double mat1_33 = mat1.back();
    mat1.pop_back();
    double mat1_32 = mat1.back();
    mat1.pop_back();
    double mat1_31 = mat1.back();
    mat1.pop_back();
    double mat1_23 = mat1.back();
    mat1.pop_back();

    double mat1_22 = mat1.back();
    mat1.pop_back();

    double mat1_21 = mat1.back();
    mat1.pop_back();

    double mat1_13 = mat1.back();
    mat1.pop_back();

    double mat1_12 = mat1.back();
    mat1.pop_back();

    double mat1_11 = mat1.back();
    mat1.pop_back();    

    double mat2_33 = mat2.back();
    mat2.pop_back();

    double mat2_32 = mat2.back();
    mat2.pop_back();

    double mat2_31 = mat2.back();
    mat2.pop_back();

    double mat2_23 = mat2.back();
    mat2.pop_back();

    double mat2_22 = mat2.back();
    mat2.pop_back();

    double mat2_21 = mat2.back();
    mat2.pop_back();

    double mat2_13 = mat2.back();
    mat2.pop_back();

    double mat2_12 = mat2.back();
    mat2.pop_back();

    double mat2_11 = mat2.back();
    mat2.pop_back();

    std::vector<double> mat;
    mat.push_back(mat1_11 - mat2_11);
    mat.push_back(mat1_12 - mat2_12);
    mat.push_back(mat1_13 - mat2_13);
    mat.push_back(mat1_21 - mat2_21);
    mat.push_back(mat1_22 - mat2_22);
    mat.push_back(mat1_23 - mat2_23);
    mat.push_back(mat1_31 - mat2_31);
    mat.push_back(mat1_32 - mat2_32);
    mat.push_back(mat1_33 - mat2_33);
    return mat;
}

double vec_norm(std::vector<double> vec){
    double vec3 = vec.back();
    vec.pop_back();
    double vec2 = vec.back();
    vec.pop_back();
    double vec1 = vec.back();
    vec.pop_back();
    double vector_norm = vec1 * vec1 + vec2 * vec2 + vec3 * vec3;
    return vector_norm;
}

std::vector<double> vec_addition(std::vector<double> u, std::vector<double> v){
    double u3 = u.back();
    u.pop_back();
    double u2 = u.back();
    u.pop_back();
    double u1 = u.back();
    u.pop_back();
    double v3 = v.back();
    v.pop_back();
    double v2 = v.back();
    v.pop_back();
    double v1 = v.back();
    v.pop_back();

    std::vector<double> vec;

    double vec1 = u1 + v1; 
    double vec2 = u2 + v2; 
    double vec3 = u3 + v3; 

    vec.push_back(vec1);
    vec.push_back(vec2);
    vec.push_back(vec3);

    return vec;
}

std::vector<double> vec_cross(std::vector<double> u, std::vector<double> v){
    double u3 = u.back();
    u.pop_back();
    double u2 = u.back();
    u.pop_back();
    double u1 = u.back();
    u.pop_back();
    double v3 = v.back();
    v.pop_back();
    double v2 = v.back();
    v.pop_back();
    double v1 = v.back();
    v.pop_back();

    std::vector<double> vec;

    double vec1 = u2 * v3 - u3 * v2; 
    double vec2 = u3 * v1 - u1 * v3; 
    double vec3 = u1 * v2 - u2 * v1; 

    vec.push_back(vec1);
    vec.push_back(vec2);
    vec.push_back(vec3);

    return vec;
}

std::vector<double> outer_product(std::vector<double> u, std::vector<double> v){
    double u3 = u.back();
    u.pop_back();
    double u2 = u.back();
    u.pop_back();
    double u1 = u.back();
    u.pop_back();
    double v3 = v.back();
    v.pop_back();
    double v2 = v.back();
    v.pop_back();
    double v1 = v.back();
    v.pop_back();

    std::vector<double> mat;

    double mat11 = u1 * v1; 
    double mat12 = u1 * v2; 
    double mat13 = u1 * v3; 
    double mat21 = u2 * v1; 
    double mat22 = u2 * v2; 
    double mat23 = u2 * v3; 
    double mat31 = u3 * v1; 
    double mat32 = u3 * v2; 
    double mat33 = u3 * v3; 
    mat.push_back(mat11);
    mat.push_back(mat12);
    mat.push_back(mat13);
    mat.push_back(mat21);
    mat.push_back(mat22);
    mat.push_back(mat23);
    mat.push_back(mat31);
    mat.push_back(mat32);
    mat.push_back(mat33);
    return mat;
}

std::vector<double> mat_vec_mult(std::vector<double> mat, std::vector<double> vec)
{
    double mat_33 = mat.back();
    mat.pop_back();

    double mat_32 = mat.back();
    mat.pop_back();

    double mat_31 = mat.back();
    mat.pop_back();

    double mat_23 = mat.back();
    mat.pop_back();

    double mat_22 = mat.back();
    mat.pop_back();

    double mat_21 = mat.back();
    mat.pop_back();

    double mat_13 = mat.back();
    mat.pop_back();

    double mat_12 = mat.back();
    mat.pop_back();

    double mat_11 = mat.back();
    mat.pop_back();

    double vec3 = vec.back();
    vec.pop_back();

    double vec2 = vec.back();
    vec.pop_back();

    double vec1 = vec.back();
    vec.pop_back();

    std::vector<double> new_vec;

    double new_vec1 = mat_11 * vec1 + mat_12 * vec2 + mat_13 * vec3;
    double new_vec2 = mat_21 * vec1 + mat_22 * vec2 + mat_23 * vec3;
    double new_vec3 = mat_31 * vec1 + mat_32 * vec2 + mat_33 * vec3;
    new_vec.push_back(new_vec1);
    new_vec.push_back(new_vec2);
    new_vec.push_back(new_vec3);

    return new_vec;


}

std::vector<double> mat_transpose(std::vector<double> mat)
{
    std::vector<double> new_mat;

    double mat_33 = mat.back();
    mat.pop_back();

    double mat_32 = mat.back();
    mat.pop_back();

    double mat_31 = mat.back();
    mat.pop_back();

    double mat_23 = mat.back();
    mat.pop_back();

    double mat_22 = mat.back();
    mat.pop_back();

    double mat_21 = mat.back();
    mat.pop_back();

    double mat_13 = mat.back();
    mat.pop_back();

    double mat_12 = mat.back();
    mat.pop_back();

    double mat_11 = mat.back();
    mat.pop_back();

    new_mat.push_back(mat_11);
    new_mat.push_back(mat_21);
    new_mat.push_back(mat_31);
    new_mat.push_back(mat_12);
    new_mat.push_back(mat_22);
    new_mat.push_back(mat_32);
    new_mat.push_back(mat_13);
    new_mat.push_back(mat_23);
    new_mat.push_back(mat_33);
    return new_mat;
}

std::vector<double> mat_multiplication(std::vector<double> mat1, std::vector<double> mat2)
{
    std::vector<double> mat;
    double mat1_33 = mat1.back();
    mat1.pop_back();
    double mat1_32 = mat1.back();
    mat1.pop_back();
    double mat1_31 = mat1.back();
    mat1.pop_back();
    double mat1_23 = mat1.back();
    mat1.pop_back();

    double mat1_22 = mat1.back();
    mat1.pop_back();

    double mat1_21 = mat1.back();
    mat1.pop_back();

    double mat1_13 = mat1.back();
    mat1.pop_back();

    double mat1_12 = mat1.back();
    mat1.pop_back();

    double mat1_11 = mat1.back();
    mat1.pop_back();    

    double mat2_33 = mat2.back();
    mat2.pop_back();

    double mat2_32 = mat2.back();
    mat2.pop_back();

    double mat2_31 = mat2.back();
    mat2.pop_back();

    double mat2_23 = mat2.back();
    mat2.pop_back();

    double mat2_22 = mat2.back();
    mat2.pop_back();

    double mat2_21 = mat2.back();
    mat2.pop_back();

    double mat2_13 = mat2.back();
    mat2.pop_back();

    double mat2_12 = mat2.back();
    mat2.pop_back();

    double mat2_11 = mat2.back();
    mat2.pop_back();

    double mat_11 = mat1_11 * mat2_11 + mat1_12 * mat2_21 + mat1_13 * mat2_31;
    double mat_12 = mat1_11 * mat2_12 + mat1_12 * mat2_22 + mat1_13 * mat2_32;
    double mat_13 = mat1_11 * mat2_13 + mat1_12 * mat2_23 + mat1_13 * mat2_33;
    double mat_21 = mat1_21 * mat2_11 + mat1_22 * mat2_21 + mat1_23 * mat2_31;
    double mat_22 = mat1_21 * mat2_12 + mat1_22 * mat2_22 + mat1_23 * mat2_32;
    double mat_23 = mat1_21 * mat2_13 + mat1_22 * mat2_23 + mat1_23 * mat2_33;
    double mat_31 = mat1_31 * mat2_11 + mat1_32 * mat2_21 + mat1_33 * mat2_31;
    double mat_32 = mat1_31 * mat2_12 + mat1_32 * mat2_22 + mat1_33 * mat2_32;
    double mat_33 = mat1_31 * mat2_13 + mat1_32 * mat2_23 + mat1_33 * mat2_33;


    mat.push_back(mat_11);
    mat.push_back(mat_12);
    mat.push_back(mat_13);
    mat.push_back(mat_21);
    mat.push_back(mat_22);
    mat.push_back(mat_23);
    mat.push_back(mat_31);
    mat.push_back(mat_32);
    mat.push_back(mat_33);


    return mat;
}

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
    : Node("coin_task_checker")
    {
        subscription_answer_ = this->create_subscription<coin_rolling_msg::msg::CoinRolling> (
                "coin/inclined_coin",
                10,
                std::bind(&MinimalSubscriber::topic_callback, this, _1)
                );
        i_ = initialize_inertia(ixx_, iyy_, izz_);

        i_world_ = i_;
    }
private:

    std::vector<double> initialize_inertia(double ixx, double iyy, double izz)
    {
        std::vector<double> inertia;
        inertia.push_back(ixx);
        inertia.push_back(0.0);
        inertia.push_back(0.0);
        inertia.push_back(0.0);
        inertia.push_back(iyy);
        inertia.push_back(0.0);
        inertia.push_back(0.0);
        inertia.push_back(0.0);
        inertia.push_back(izz);
        return inertia;
    }

    void topic_callback(const coin_rolling_msg::msg::CoinRolling::SharedPtr msg) const
    {

        geometry_msgs::msg::Pose msg_pose = msg->pose;
        geometry_msgs::msg::Twist msg_twist = msg->twist;

        double w = msg_pose.orientation.w;
        double x = msg_pose.orientation.x;
        double y = msg_pose.orientation.y;
        double z = msg_pose.orientation.z;

        double xlin = msg_pose.position.x;
        double ylin = msg_pose.position.y;
        double zlin = msg_pose.position.z;

        std::vector<double> position_vector;
        position_vector.push_back(xlin);
        position_vector.push_back(ylin);
        position_vector.push_back(zlin);

        double R11 = 1.0 - 2.0*y*y - 2.0*z*z;
        double R12 = 2.0*x*y - 2.0*w*z;
        double R13 = 2.0*x*z + 2.0*w*y;

        double R21 = 2.0*x*y + 2.0*w*z;
        double R22 = 1.0 - 2.0*x*x - 2.0*z*z;
        double R23 = 2.0*y*z - 2.0*w*x;

        double R31 = 2.0*x*z - 2.0*w*y;
        double R32 = 2.0*y*z + 2.0*w*x;
        double R33 = 1.0 - 2.0*x*x - 2.0*y*y;
        std::vector<double> R;

        R.push_back(R11);
        R.push_back(R12);
        R.push_back(R13);
        R.push_back(R21);
        R.push_back(R22);
        R.push_back(R23);
        R.push_back(R31);
        R.push_back(R32);
        R.push_back(R33);

        std::vector<double> R_transpose = mat_transpose(R);
        std::vector<double> i_body_in_world_frame = mat_multiplication(mat_multiplication(R, i_), R_transpose);

        std::vector<double> linvel;
        std::vector<double> angvel;

        linvel.push_back(msg_twist.linear.x);
        linvel.push_back(msg_twist.linear.y);
        linvel.push_back(msg_twist.linear.z);

        angvel.push_back(msg_twist.angular.x);
        angvel.push_back(msg_twist.angular.y);
        angvel.push_back(msg_twist.angular.z);      

        std::vector<double> answer_angular_momentum_body = mat_vec_mult(i_body_in_world_frame, angvel);
        std::vector<double> answer_angular_momentum_center = std_vec_const_mult(vec_cross(position_vector, linvel), mass_);
        std::vector<double> answer_angular_momentum = vec_addition(answer_angular_momentum_center, answer_angular_momentum_body);

        double answer_angular_momentum_z = answer_angular_momentum.back();
        answer_angular_momentum.pop_back();
        double answer_angular_momentum_y = answer_angular_momentum.back();
        answer_angular_momentum.pop_back();
        double answer_angular_momentum_x = answer_angular_momentum.back();
        answer_angular_momentum.pop_back();

        double test_angular_momentum_x = msg->angular_momentum.x;
        double test_angular_momentum_y = msg->angular_momentum.y;
        double test_angular_momentum_z = msg->angular_momentum.z;

        bool condition_x = false;
        bool condition_y = false;
        bool condition_z = false;

        if (fabs((test_angular_momentum_x - answer_angular_momentum_x) / answer_angular_momentum_x) < 0.05)
            condition_x = true;

        if (fabs((test_angular_momentum_y - answer_angular_momentum_y) / answer_angular_momentum_y) < 0.05)
            condition_y = true;

        if (fabs((test_angular_momentum_z - answer_angular_momentum_z) / answer_angular_momentum_z) < 0.05)
            condition_z = true; 
        if (condition_x & condition_y & condition_z)
            RCLCPP_INFO(this->get_logger(), "Correct answer");
        else if (condition_x & condition_y)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum z");
        else if (condition_x & condition_z)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum y");
        else if (condition_y & condition_z)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum x");
        else if (condition_x)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum y, z");
        else if (condition_y)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum x, z");
        else if (condition_z)
            RCLCPP_INFO(this->get_logger(), "Wrong answer in momentum x, y"); 
        else
            RCLCPP_INFO(this->get_logger(), "Wrong answer in total momentum");
    }

    rclcpp::Subscription<coin_rolling_msg::msg::CoinRolling>::SharedPtr subscription_answer_;

    double mass_ = 10;
    double ixx_ = 0.7;
    double iyy_ = 0.7;
    double izz_ = 1.25;
    std::vector<double> i_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();

    return 0;
}
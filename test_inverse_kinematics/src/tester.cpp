#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "inverse_kinematics/inverse_kinematics.hpp"

using namespace std::chrono_literals;

class Tester : public rclcpp::Node
{
    public:
    Tester()
    : Node("tester"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("tester_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&Tester::timer_callback, this));
    }

    private:
    // initialisation for the inverse kinematics function
    const float body_dimensions_[2] = {1,1};
    const float leg_dimensions_[2][3] = {
        {2, 0, 0},
        {2, 0, 0},
    };
    const float base_frame_height_ = 0.5;
    const float distance_between_hip_joints_ = 0;

    Eigen::VectorXf q_0 = Eigen::Matrix<float,12,1>::Zero();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    
    void timer_callback()
    {
        Eigen::Vector3f I_r_IE_des = {0, -1, -2};
        vector<int> stationary_feet = {1,0,0,0};
        Eigen::Vector3f body_orientation = Eigen::Vector3f::Zero();
        
        auto message = std_msgs::msg::String();
        InverseKinematics IK = InverseKinematics(body_dimensions_, leg_dimensions_, base_frame_height_, distance_between_hip_joints_);
        Eigen::VectorXf q_dot = IK.inverse_kinematics(q_0, I_r_IE_des, stationary_feet, body_orientation, Eigen::Matrix3f::Zero());
        //q_0 = q_dot;

        message.data = "q_dot: \n";
        int counter = 0;
        for (int i = 0; i < 18; i++) 
        {
            counter++;
            message.data += std::to_string(q_dot(i)) + "\n";
            if (counter == 3)
            {
                message.data += "\n";
                counter = 0;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Publishing q_dot: \n'%s'", message.data.c_str());
        publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Tester>());
    rclcpp::shutdown();
}
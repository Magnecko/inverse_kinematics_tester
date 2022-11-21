#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "inverse_kinematics/inverse_kinematics.hpp"
#define PI 3.1415926

using namespace std::chrono_literals;

class Tester : public rclcpp::Node
{
    public:
    Tester()
    : Node("tester"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("tester_topic", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&Tester::timer_callback, this));
        std::cout << q_0;
    }

    private:
    // initialisation for the inverse kinematics function
    const double body_dimensions_[2] = {1,1};
    const double leg_dimensions_[2][3] = {
        {2, 0, 0},
        {2, 0, 0},
    };
    const double base_frame_height_ = 0.5;
    const double distance_between_hip_joints_ = 0;
    Eigen::VectorXd q_pos = Eigen::Vector<double,18>::Zero();
    Eigen::VectorXd q_0 = Eigen::Vector<double,18>::Zero();
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int counter = 0;
    InverseKinematics IK = InverseKinematics(body_dimensions_, leg_dimensions_, base_frame_height_, distance_between_hip_joints_);
    
    void timer_callback()
    {
        Eigen::Vector3d I_r_IE_des = {0, -1, -2};
        vector<int> stationary_feet = {1,0,0,0};
        Eigen::Vector3d body_orientation = Eigen::Vector3d::Zero();
        
        auto message = std_msgs::msg::String();
        
        Eigen::VectorXd q_dot = IK.inverse_kinematics(q_0, I_r_IE_des, stationary_feet, body_orientation, Eigen::Matrix3d::Zero());
        if (counter < 1000)
        {
            q_pos = q_pos + q_dot*0.1;
                //Normalize the angles to their limits
            for(auto &i : q_pos){
                int n = int(abs(i)/2/PI + 0.5);
                i > 0 ? i -= n*2*PI: i += n*2*PI;
            }
            q_0 = q_pos;
        }
        counter++;
        message.data = "q_dot: \n";
        int counter = 0;
        for (int i = 0; i < 18; i++) 
        {
            counter++;
            message.data += std::to_string(q_pos(i)) + "\n";
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
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>

#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include "OmniDriveSystem.hpp"
#include "DriveLayout.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotinoNode : public rclcpp::Node
{
public:
    RobotinoNode()
        : Node("robotino_node"),
        _omni(DriveLayout())
    {
        this->declare_parameter("motor/max_rpm", 0);
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&RobotinoNode::vel_callback, this, _1));
        
        joint_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotinoNode::joint_state_callback, this, std::placeholders::_1)
        );

        wheel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/velocity_controller/commands", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        motorEN = true;
        last_wheel_time_ = get_clock()->now();
    }

private:

    struct LocPose {
        double x = 0, y = 0, theta = 0;
    } pose_local_;
    void vel_callback(const geometry_msgs::msg::Twist msg)
    {
        if(motorEN){
            auto message = std_msgs::msg::Float64MultiArray();
            float vels[] = {0,0,0,0};
            _omni.projectVelocity(vels, maxRPM, 
                msg.linear.x, msg.linear.y, msg.angular.z);
            
            message.data = {-vels[0], -vels[1], -vels[2]};

            wheel_publisher_->publish(message);
        }
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Cria um mapa para associar cada nome de junta com sua velocidade
        std::unordered_map<std::string, double> joint_velocity_map;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            joint_velocity_map[msg->name[i]] = msg->velocity[i];
        }

        double m1 = -joint_velocity_map[joint_names_[0]];
        double m2 = -joint_velocity_map[joint_names_[1]];
        double m3 = -joint_velocity_map[joint_names_[2]];

        

        float vx, vy, omega;
        _omni.unprojectVelocity(&vx, &vy, &omega, m1, m2, m3, 0);

        rclcpp::Time msg_time(msg->header.stamp);

        last_wheel_time_ = msg_time;

        float dt = (msg_time - last_wheel_time_).seconds();

        integrateByRungeKutta(vx, vy, omega, dt);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        // Preenche a pose na mensagem de odometria
        odom_msg.pose.pose.position.x = pose_local_.x;
        odom_msg.pose.pose.position.y = pose_local_.y;

        tf2::Quaternion q;
        q.setRPY(0, 0, pose_local_.theta);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();

        // Preenche a velocidade na mensagem de odometria
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = omega;

        // Publica a mensagem de odometria
        odom_publisher_->publish(odom_msg);

    }

    void integrateByRungeKutta(float vx, float vy, float wz, float dt_) {
        double theta_bar = pose_local_.theta + (wz*dt_ / 2.0f);
        pose_local_.x = pose_local_.x + (vx * cos(theta_bar) - vy * sin(theta_bar)) * dt_;
        pose_local_.y = pose_local_.y + (vx * sin(theta_bar) + vy * cos(theta_bar)) * dt_;
        pose_local_.theta += + pose_local_.theta*dt_;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Time last_wheel_time_;

    bool motorEN = false;
    OmniDriveSystem _omni;
    std::vector<std::string> joint_names_ = {"omni_left_joint", "omni_back_joint", "omni_right_joint"};
    const float maxRPM[4] = {10000000.0f, 10000000.0f, 10000000.0f};
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotinoNode>());
    rclcpp::shutdown();
    return 0;
}
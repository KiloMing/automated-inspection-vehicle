#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class InitialPosePublisher : public rclcpp::Node
{
    public:
        InitialPosePublisher() : Node("initial_pose_publisher")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
            timer_ = this->create_wall_timer(
                3s,
                [this]()
                {
                    publisher_initial_pose();
                }
            );
        }

    private:
        void publisher_initial_pose()
        {
            geometry_msgs::msg::PoseWithCovarianceStamped msg;

            // 1, Header
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";

            // 2, initial pose
            msg.pose.pose.position.x = -1.0;
            msg.pose.pose.position.y = 3.66;
            msg.pose.pose.position.z = 0.0;

            // 3, yaw
            double yaw = 1.0503;
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            msg.pose.pose.orientation = tf2::toMsg(q);

            // 4, pose covariance
            msg.pose.covariance[0] = 0.25;  // x
            msg.pose.covariance[7] = 0.25;  // y
            msg.pose.covariance[35] = 0.25; // yaw

            // 5, publisher
            RCLCPP_INFO(
                this->get_logger(), 
                "Initial pose publised: x=%.2f y=%.2f yaw=%.2f", 
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y, 
                yaw
            );

            // just publish once 
            publisher_->publish(msg);

            timer_->cancel();
        }

    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

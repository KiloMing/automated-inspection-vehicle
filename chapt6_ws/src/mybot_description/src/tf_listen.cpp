#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.h"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TFListenerNode : public rclcpp::Node
{
    public:
        TFListenerNode() : Node("tf_listener_node")
        {
            tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = this->create_wall_timer(
                500ms,
                [this](){
                    query_transform();
                }
            );
        }
    
    private: 
        void query_transform()
        {
            try
            {
                auto transform = tf_buffer_->lookupTransform(
                    "map",
                    "base_footprint",
                    tf2::TimePointZero
                );
 
                tf2::Quaternion q;
                tf2::fromMsg(transform.transform.rotation, q);
                double roll;
                double pitch;
                double yaw;

                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

                RCLCPP_INFO(
                    this->get_logger(),
                    "x = %.3f, y = %.2f, z = %.2f, roll = %.2f, pitch = %.2f, yaw = %.2f",
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z,
                    roll,
                    pitch,
                    yaw
                );
            }
            catch(const tf2::TransformException & ex)
            {
                RCLCPP_WARN(this->get_logger(), "TF query failed: %s", ex.what());
            }
        }

    private:
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TFListenerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
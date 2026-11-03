#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <memory>
using namespace std::chrono_literals;

class NavigateToPoseClient : public rclcpp::Node
{
    public:
        using NavigateToPose = nav2_msgs::action::NavigateToPose;
        using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

        NavigateToPoseClient():Node("navigate_to_pose_client")
        {
            // 1, action client
            action_client_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose"
            );

            // 2, timer
            timer_ = this->create_wall_timer(
                2s, 
                [this]()
                {
                    timer_->cancel();
                    send_goal();
                }
            );
            RCLCPP_INFO(this->get_logger(), "NavigateToPose client started. ");
        }

    private:
        void send_goal()
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for /navigate_to_pose Action server...");
            if(!action_client_->wait_for_action_server(5s))
            {
                RCLCPP_ERROR(this->get_logger(), "NavigateToPose Action Server is not avaiable: ");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "NavigateToPose Action Server connected. ");

            // create goal Pose
            NavigateToPose::Goal goal_msg;

            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->now();
            
            // set goal pose example:x = 1.637, y = 5.17, z = 0.00, roll = 0.00, pitch = -0.00, yaw = -2.72
            goal_msg.pose.pose.position.x = 1.637;
            goal_msg.pose.pose.position.y = 5.17;
            goal_msg.pose.pose.position.z = 0.0;

            // set goal Yaw 
            //constexpr double PI = 3.14159265358979323846;
            //double target_yaw_degree = 90.0;
            //double target_yaw = target_yaw_degree * PI / 180.0;
            double target_yaw = -2.72;
            tf2::Quaternion quaternion;
            quaternion.setRPY(0.0, 0.0, target_yaw);
            goal_msg.pose.pose.orientation = tf2::toMsg(quaternion);  //tf2 Quaternion -> geometry_msgs Quaternion

            // behavior_tree : 默认使用Behavior Tree
            goal_msg.behavior_tree = "";
            RCLCPP_INFO(
                this->get_logger(),
                "Sending navigation goal:"
            );

            RCLCPP_INFO(
                this->get_logger(),
                "x = %.2f, y = %.2f, yaw = %.2f",
                goal_msg.pose.pose.position.x,
                goal_msg.pose.pose.position.y,
                target_yaw
            );

            // Action callback
            auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

            send_goal_options.goal_response_callback = [this](GoalHandleNavigateToPose::SharedPtr goal_handle)
            {
                if(!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal rejected by Nav2.");
                }
                else
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Goal accepted by Nav2."
                    );
                }
            };
            // Feedback Callback
            send_goal_options.feedback_callback = [this](GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
            {
                double current_x = feedback->current_pose.pose.position.x;
                double current_y = feedback->current_pose.pose.position.y;
                RCLCPP_INFO(this->get_logger(), "Current position: x = %.3f, y = %.3f", current_x, current_y);
            };

            //Result Callback 
            send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result)
            {
                switch (result.code)
                {
                    //successsful
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
                        break;
                    
                    //break 
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Navigation aborted !");
                        break;

                    //cancled
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Navigation canceled");
                        break;
                    //unknown
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown navigation rusult");
                }
                if(result.result)
                {
                    RCLCPP_INFO(this->get_logger(), "Nav2 error_code: %u", result.result->error_code);
                    if(!result.result->error_msg.empty())
                    {
                        RCLCPP_INFO(this->get_logger(), "Nav2 error_msg : %s", result.result->error_msg.c_str());
                    }
                }
            };
            action_client_->async_send_goal(goal_msg, send_goal_options);
        }
    private:
        // action client
        rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
        // timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateToPoseClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


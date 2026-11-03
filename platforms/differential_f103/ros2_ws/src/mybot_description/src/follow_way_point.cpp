#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;


class FollowWayPointsClient : public rclcpp::Node
{
    public:
        using FollowWayPoints = nav2_msgs::action::FollowWaypoints;
        using GoalHandleFollowWayPoints = rclcpp_action::ClientGoalHandle<FollowWayPoints>;

        FollowWayPointsClient():Node("navigate_to_pose_client")
        {
            // 1, action client
            action_client_ = rclcpp_action::create_client<FollowWayPoints>(
                this,
                "/follow_waypoints"
            );

            // 2, timer
            timer_ = this->create_wall_timer(
                2s, 
                [this]()
                {
                    send_goal();
                }
            );
            RCLCPP_INFO(this->get_logger(), "FollowWayPoints client started. ");
        }

    private:
        geometry_msgs::msg::PoseStamped create_pose(double x, double y, double yaw)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();

            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            return pose;
        }


        void send_goal()
        {

            RCLCPP_INFO(this->get_logger(), "Waiting for /follow_waypoints Action server...");
            if(!action_client_->wait_for_action_server(5s))
            {
                RCLCPP_ERROR(this->get_logger(), "FollowWayPoints Action Server is not avaiable: ");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "FollowWayPoints Action Server connected. ");
            timer_->cancel();

            //setting goal_msg
            FollowWayPoints::Goal goal_msg;
            goal_msg.number_of_loops = 3;   // Number of cruises 
            goal_msg.goal_index = 0;        // start at the index


            // create goal Pose
            goal_msg.poses.push_back(create_pose(-0.507, -0.09, 1.62));
            goal_msg.poses.push_back(create_pose(-3.14, 3.11, 1.15));
            goal_msg.poses.push_back(create_pose(-0.497, 6.65, -1.94));
            goal_msg.poses.push_back(create_pose(1.616, 3.33, 2.80));
            // Action callback
            rclcpp_action::Client<FollowWayPoints>::SendGoalOptions send_goal_options;

            send_goal_options.goal_response_callback = [this](GoalHandleFollowWayPoints::SharedPtr goal_handle)
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
            send_goal_options.feedback_callback = [this](GoalHandleFollowWayPoints::SharedPtr, const std::shared_ptr<const FollowWayPoints::Feedback> feedback)
            {
                RCLCPP_INFO(this->get_logger(), "Current waypoint %u", feedback->current_waypoint);
            };

            //Result Callback 
            send_goal_options.result_callback = [this](const GoalHandleFollowWayPoints::WrappedResult & result)
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
        rclcpp_action::Client<FollowWayPoints>::SharedPtr action_client_;
        // timer
        rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowWayPointsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


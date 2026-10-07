#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "autopatrol_interface/srv/speech_text.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>
#include <memory>
using namespace std::chrono_literals;

class PatrolController : public rclcpp::Node
{
    public:
        using FollowWayPoints = nav2_msgs::action::FollowWaypoints;
        using GoalHandleFollowWayPoints = rclcpp_action::ClientGoalHandle<FollowWayPoints>;
        using SpeechText = autopatrol_interface::srv::SpeechText;

        PatrolController():Node("navigate_to_pose_client")
        {
            patrol_points_ = {
                {"A", -0.507, -0.09, 1.62},
                {"B",-3.14, 3.11, 1.15},
                {"C",-0.497, 6.65, -1.94},
                {"D",1.616, 3.33, 2.80}
            };

            //photo client
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image",
                10,
                [this](sensor_msgs::msg::Image::SharedPtr msg)
                {
                    image_callback(msg);
                }
            );
            
            //speech client
            speech_client_ = this->create_client<SpeechText>("/speech_text");
            
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
        //for photo
        void image_callback(sensor_msgs::msg::Image::SharedPtr msg)
        {
            latest_image_ = msg;
            RCLCPP_DEBUG(this->get_logger(), "Received image: %u x %u", msg->width, msg->height);
        }

        void save_current_image(const std::string & point_name)
        {
            if(!latest_image_)
            {
                RCLCPP_WARN(this->get_logger(), "No camera iamge received yet.");
                return;
            }

            try
            {
                auto cv_ptr = cv_bridge::toCvCopy(latest_image_, "bgr8");
                std::string filename = "/home/kiloming/my_ros/chapt6_ws/photo/" + point_name + ".jpg";
                bool success = cv::imwrite(filename, cv_ptr->image);
                if(success)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Image saved: %s",
                        filename.c_str()
                    );
                }
                 else
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Failed to save image."
                    );
                }
            }
            catch (const cv_bridge::Exception & e)
            {
                RCLCPP_ERROR(
                    this->get_logger(),
                    "cv_bridge error: %s",
                    e.what()
                );
            }

        }

        //for speech client
        void speak(const std::string &text)
        {
            if(!speech_client_->wait_for_service(1s))
            {
                RCLCPP_WARN(this->get_logger(), "Speech service is not available.");
                return;
            }
            auto request = std::make_shared<SpeechText::Request>();
            request->text = text;
            speech_client_->async_send_request(request, 
                [this](rclcpp::Client<SpeechText>::SharedFuture future){
                    auto response = future.get();
                    if(response->result)
                    {
                        RCLCPP_INFO(this->get_logger(), "Speech succeeded.");
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Speech failed");
                    }
                }
            );
        }


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

        void on_waypoint_reached(uint32_t index)
        {
            if(index >= patrol_points_.size())
            {
                return;
            }
            // log the point
            const auto & point = patrol_points_[index];
            RCLCPP_INFO(this->get_logger(), "Reached patrol point %u %s", index, point.name.c_str());

            //speak 
            std::string speech_text = "Reached " + point.name;
            speak(speech_text);
            //photo
            save_current_image(point.name);
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
            for (const auto & point : patrol_points_)
            {
                goal_msg.poses.push_back(create_pose(point.x, point.y, point.yaw));
            }
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
                current_waypoint_ = feedback->current_waypoint;
                if (first_feedback_)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Start patrolling waypoint %u",
                        current_waypoint_
                    );

                    last_waypoint_ = current_waypoint_;
                    first_feedback_ = false;
                    return;
                }

                if(current_waypoint_ != last_waypoint_)
                {
                    on_waypoint_reached(last_waypoint_);
                    RCLCPP_INFO(this->get_logger(), "Waypoint changed: %u -> %u", last_waypoint_, current_waypoint_);
                    last_waypoint_ = current_waypoint_;
                }
            };

            //Result Callback 
            send_goal_options.result_callback = [this](const GoalHandleFollowWayPoints::WrappedResult & result)
            {
                switch (result.code)
                {
                    //successsful
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
                        // for the last point
                        if(!patrol_points_.empty())
                        {
                            on_waypoint_reached(static_cast<uint32_t>(patrol_points_.size()-1));
                        }

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
        struct PatrolPoint
        {
            std::string name;
            double x;
            double y;
            double yaw;
        };

        // action client
        rclcpp_action::Client<FollowWayPoints>::SharedPtr action_client_;
        // timer
        rclcpp::TimerBase::SharedPtr timer_;
        uint32_t last_waypoint_{0};
        uint32_t current_waypoint_{0};
        bool first_feedback_{true};
        std::vector<PatrolPoint> patrol_points_;
        rclcpp::Client<SpeechText>::SharedPtr speech_client_;

        //photo 
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
        sensor_msgs::msg::Image::SharedPtr latest_image_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


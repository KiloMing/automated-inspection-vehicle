#include "rclcpp/rclcpp.hpp"
#include "autopatrol_interface/srv/speech_text.hpp"
#include <cstdlib>
#include <string>
#include <memory>

using SpeechText = autopatrol_interface::srv::SpeechText;

class SpeakerNode : public rclcpp :: Node 
{
    public:
        SpeakerNode() : Node("speaker")
        {
            service_ = this->create_service<SpeechText>(
                "/speech_text",
                [this](
                    const std::shared_ptr<SpeechText::Request> request, 
                    std::shared_ptr<SpeechText::Response> response)
                    {
                        speech_callback(request, response);
                    }
                );
            RCLCPP_INFO(this->get_logger(), "Speaker service started.");
        }

    private:
        void speech_callback(const std::shared_ptr<SpeechText::Request> request, std::shared_ptr<SpeechText::Response> response)
        {
            RCLCPP_INFO(this->get_logger(), "Received text : %s", request->text.c_str());

            std::string command = "espeak-ng \"" + request->text + "\"";
            int result = std::system(command.c_str());
            
            if(result == 0)
            {
                response->result = true;
                RCLCPP_INFO(this->get_logger(), "Speech completed.");
            }
            else
            {
                response->result = false;
                RCLCPP_ERROR(this->get_logger(), "Speech failed");
            }

            response->result = true;
        }
        
    private:
        rclcpp::Service<SpeechText>::SharedPtr service_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SpeakerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
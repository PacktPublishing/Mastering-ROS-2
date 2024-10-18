#include "rclcpp/rclcpp.hpp"
#include "master_ros2_interface/srv/concat_strings.hpp"

class StringConcatService : public rclcpp::Node
{
public:
    StringConcatService()
    : Node("string_concat_service")
    {

        RCLCPP_INFO(this->get_logger(), "Started ROS 2 Service Server");

        service_ = this->create_service<master_ros2_interface::srv::ConcatStrings>(
            "concat_strings",
            std::bind(&StringConcatService::handle_service, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_service(
        const std::shared_ptr<master_ros2_interface::srv::ConcatStrings::Request> request,
        std::shared_ptr<master_ros2_interface::srv::ConcatStrings::Response> response)
    {
        response->concatenated_str = request->str1 + request->str2;
        RCLCPP_INFO(this->get_logger(), "Received: str1='%s', str2='%s'; Responding with: '%s'", 
                    request->str1.c_str(), request->str2.c_str(), response->concatenated_str.c_str());
    }

    rclcpp::Service<master_ros2_interface::srv::ConcatStrings>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StringConcatService>());
    rclcpp::shutdown();
    return 0;
}


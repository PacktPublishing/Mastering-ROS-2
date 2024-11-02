#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>

class ToLowercase : public rclcpp::Node {
public:
    ToLowercase()
    : Node("lowercase_string_node") {        
        _input_sub = this->create_subscription<std_msgs::msg::String>("input", 10, std::bind(&ToLowercase::input_str_cb, this, std::placeholders::_1));        
        _output_pub = this->create_publisher<std_msgs::msg::String>("output", 10);
    }
    void input_str_cb(const std_msgs::msg::String::SharedPtr msg);

private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _input_sub;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _output_pub;
};

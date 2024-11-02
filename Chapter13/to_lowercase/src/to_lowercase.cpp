#include "to_lowercase.h"


void ToLowercase::input_str_cb(const std_msgs::msg::String::SharedPtr msg) {
    std::string lowercase_str = msg->data;
    std::transform(lowercase_str.begin(), lowercase_str.end(), lowercase_str.begin(), ::tolower);        
    auto message = std_msgs::msg::String();
    message.data = lowercase_str;        
    _output_pub->publish(message);
}




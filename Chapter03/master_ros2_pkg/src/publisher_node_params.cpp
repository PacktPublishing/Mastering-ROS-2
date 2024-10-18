#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "master_ros2_interface/msg/custom_msg.hpp"

class PublisherNode : public rclcpp::Node {
public:
    PublisherNode() : Node("publisher_node_params") {
        // Declare string and integer parameters
        this->declare_parameter<std::string>("custom_string", "Hello World");
        this->declare_parameter<int>("custom_number", 42);

        // QoS profiles for both publishers
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10))    // History QoS (Keep last 10 messages)
                               .reliable()                      // Reliability QoS (reliable communication)
                               .transient_local();              // Durability QoS (message remains available to late subscribers)

        // Publisher for std_msgs::msg::String with QoS settings
        publisher_ = this->create_publisher<std_msgs::msg::String>("std_string_topic", qos_profile);

        // Publisher for custom message master_ros2_interface::msg::CustomMsg with QoS settings
        custom_publisher_ = this->create_publisher<master_ros2_interface::msg::CustomMsg>("custom_topic", qos_profile);

        // Timer to publish messages periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&PublisherNode::publish_messages, this));
    }

private:
    // Method to publish both standard and custom messages
    void publish_messages() {
        // Publish std_msgs::msg::String
        auto string_msg = std_msgs::msg::String();
        string_msg.data = "Hello, world!";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", string_msg.data.c_str());
        publisher_->publish(string_msg);

        // Get parameter values
        std::string custom_string_param;
        int custom_number_param;

        this->get_parameter("custom_string", custom_string_param);
        this->get_parameter("custom_number", custom_number_param);

        // Publish custom message master_ros2_interface::msg::CustomMsg
        auto custom_msg = master_ros2_interface::msg::CustomMsg();
        custom_msg.data = custom_string_param;  // Use parameter value for string
        custom_msg.number = custom_number_param;  // Use parameter value for number
        RCLCPP_INFO(this->get_logger(), "Publishing custom message: data='%s', number=%d", custom_msg.data.c_str(), custom_msg.number);
        custom_publisher_->publish(custom_msg);

        // Set parameters programmatically
        this->set_parameter(rclcpp::Parameter("custom_string", "Hello String"));
        this->set_parameter(rclcpp::Parameter("custom_number", 42));


        
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<master_ros2_interface::msg::CustomMsg>::SharedPtr custom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PublisherNode>());
    rclcpp::shutdown();
    return 0;
}


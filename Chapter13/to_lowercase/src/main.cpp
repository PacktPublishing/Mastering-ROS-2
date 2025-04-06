#include "to_lowercase.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToLowercase>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

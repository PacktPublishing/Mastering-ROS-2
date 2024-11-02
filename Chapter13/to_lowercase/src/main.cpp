#include "to_lowercase.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToLowercase>());
    rclcpp::shutdown();
    return 0;
}

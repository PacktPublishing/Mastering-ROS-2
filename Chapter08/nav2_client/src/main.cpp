#include "nav2_client/navigation_client.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto navigator = std::make_shared<NavigationClient>();
    
    while (rclcpp::ok()) {
        std::string input;
        std::cout << "Enter order number (1 for Room1, 2 for Room2, q to quit): ";
        std::getline(std::cin, input);
        
        if (input == "q" || input == "Q") {
            break;
        }
        
        if (input != "1" && input != "2") {
            std::cout << "Invalid order number. Please enter 1 or 2." << std::endl;
            continue;
        }
        
        try {
            navigator->deliverMedicine(std::stoi(input));
            rclcpp::spin_some(navigator);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(navigator->get_logger(), "Error during navigation: %s", e.what());
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <gtest/gtest.h>
#include "../src/to_lowercase.h"

TEST(package_name, UpperToLower) {

  auto to_lowercase_node = std::make_shared<ToLowercase>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(to_lowercase_node);

  bool ready = false;
  std::string value;

  auto output_callback = [this, &ready, &value](const std_msgs::msg::String::SharedPtr msg) {
    value = msg->data;
    ready = true;
  };

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr output_sub;
  output_sub = to_lowercase_node->create_subscription<std_msgs::msg::String>("/output", 10, output_callback);
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub;
  string_pub = to_lowercase_node->create_publisher<std_msgs::msg::String>("/input", 10);

  std_msgs::msg::String s;
  s.data = "TOLOWER";
  string_pub->publish( s );

  rclcpp::Rate rate(5);  // Set rate to 1 Hz (once per second)
  while( !ready ) {
    executor.spin_once(std::chrono::milliseconds(100)); // Process callbacks with a timeout of 100 ms
    rate.sleep();
  }  
  ASSERT_EQ(value, "tolower");
  
  ready = false;
  s.data = "TO-LOWER-2";
  string_pub->publish( s );  
  while( !ready ) {
    executor.spin_once(std::chrono::milliseconds(100)); // Process callbacks with a timeout of 100 ms
    rate.sleep();
  }  
  
  ASSERT_EQ(value, "to-lower-2");
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);


  rclcpp::init(argc, argv);

  
  return RUN_ALL_TESTS();
}
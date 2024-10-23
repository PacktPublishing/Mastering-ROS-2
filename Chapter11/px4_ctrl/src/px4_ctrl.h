#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <boost/thread.hpp>  
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;


class Px4Control : public rclcpp::Node {
    public:
        Px4Control()  : Node("px4_ctrl") {
            _cmd_client = create_client<px4_msgs::srv::VehicleCommand>("/fmu/vehicle_command");
            _traj_cmd_pub = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            _offboard_ctrl_mode_publisher= create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/in/offboard_control_mode", 10);

            while (!_cmd_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		    }
            rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
            
            // Setting QoS attributes
            qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
            qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
            qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
            qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);  // Depth could be set here
            qos_profile.keep_last(10);  

            _uav_pose_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>("/fmu/out/vehicle_local_position", qos_profile,  // QoS of 10
            std::bind(&Px4Control::uav_pose_cb, this, std::placeholders::_1));

            _action_done = false;
        }
        void change_mode();
        void arm();
        void disarm();
        void uav_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg); 
        void srv_callback( rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future );
        void send_cmd(uint16_t command, float param1, float param2);
        void publish_offboard_ctrl_mode();
        void run();
        void menu();
 
    private:
        rclcpp::Publisher<OffboardControlMode>::SharedPtr        _offboard_ctrl_mode_publisher;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr         _setpoint_publisher;
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr         _traj_cmd_pub;
        rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr _cmd_client;

        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _uav_pose_sub;
        bool _action_done;

        float _curr_x;
        float _curr_y;
        float _curr_z;

};  

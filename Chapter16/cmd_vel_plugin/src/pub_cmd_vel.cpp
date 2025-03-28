#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <gz/sim/System.hh>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <sdf/sdf.hh>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/details/twist.pb.h>


namespace cmd_vel_plugin {
    class PubCmdVel:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate {
    
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    public: void Configure(const gz::sim::Entity &_id,
                            const std::shared_ptr<const sdf::Element> &_sdf,
                            gz::sim::EntityComponentManager &_ecm,
                            gz::sim::EventManager &_eventMgr) final;
    public: void cmd_vel_cb( const geometry_msgs::msg::Twist );

    private: 
      rclcpp::Node::SharedPtr ros_node_; 
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ros_subscriber_;
      gz::transport::Node::Publisher gz_publisher_;
       gz::transport::Node gz_node_;
     gz::msgs::Twist cmd_vel_msg_;

  };
}


GZ_ADD_PLUGIN (
    cmd_vel_plugin::PubCmdVel,
    gz::sim::System,
    cmd_vel_plugin::PubCmdVel::ISystemConfigure,
    cmd_vel_plugin::PubCmdVel::ISystemPostUpdate)

using namespace cmd_vel_plugin;

void PubCmdVel::cmd_vel_cb( const geometry_msgs::msg::Twist t) {
    double vx = t.linear.x;
    if( vx > 0.2) vx = 0.2;
    else if (vx < -0.2) vx = -0.2;
    cmd_vel_msg_.mutable_linear()->set_x (vx);
    
    double vz = t.linear.z;
    if( vz > 0.2) vz = 0.2;
    else if (vz < -0.2) vz = -0.2;
    cmd_vel_msg_.mutable_linear()->set_z (vz);
    
    double vrot_z = t.angular.z;
    if( vrot_z > 0.5 ) vrot_z = 0.5;
    else if (vrot_z < -0.5) vrot_z = 0.5;
    cmd_vel_msg_.mutable_angular()->set_z(vrot_z);
 
}


void PubCmdVel::Configure(const gz::sim::Entity &_entity, 
    const std::shared_ptr<const sdf::Element> &_sdf, gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &/*_eventMgr*/) {

    rclcpp::init(0, nullptr);
    ros_node_ = rclcpp::Node::make_shared("cmd_vel_plugin");
    ros_subscriber_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_from_ros", 10, std::bind(&PubCmdVel::cmd_vel_cb, this, std::placeholders::_1));
    gz_publisher_ = gz_node_.Advertise<gz::msgs::Twist>("/cmd_vel");
}
void PubCmdVel::PostUpdate(const gz::sim::UpdateInfo &_info, const gz::sim::EntityComponentManager &/*_ecm*/) {    
    rclcpp::spin_some( ros_node_ );
    gz_publisher_.Publish(cmd_vel_msg_);
}


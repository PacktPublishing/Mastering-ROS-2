#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class CartesianPlanning : public rclcpp::Node
{
public:
    CartesianPlanning() : Node("cartesian_planning") {}
    void run();
    void plan();
private:
  rclcpp::Node::SharedPtr _node;
};

void CartesianPlanning::plan() {
    bool tf_found = false;    
    geometry_msgs::msg::TransformStamped t;

    auto tf_buffer {std::make_unique<tf2_ros::Buffer>(this->get_clock())};
    auto tf_listener { std::make_shared<tf2_ros::TransformListener>(*tf_buffer)};

    //std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    //std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    //tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    //tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    moveit::planning_interface::MoveGroupInterface move_group(_node, "arm");
    std::string fromFrameRel = move_group.getPlanningFrame().c_str();
    std::string toFrameRel = move_group.getEndEffectorLink().c_str();

    while ( !tf_found ) {    
        try {
            t = tf_buffer->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
            tf_found = true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s Try again",toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            sleep(1);
        }
    }

    geometry_msgs::msg::Pose target_pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    target_pose.orientation.w = t.transform.rotation.w; target_pose.orientation.x = t.transform.rotation.x; target_pose.orientation.y = t.transform.rotation.y;
    target_pose.orientation.z = t.transform.rotation.z; target_pose.position.x = t.transform.translation.x; target_pose.position.y = t.transform.translation.y;
    target_pose.position.z = t.transform.translation.z;
    waypoints.push_back(target_pose);
    target_pose.position.z -= 0.2;
    waypoints.push_back(target_pose);  
    target_pose.position.y -= 0.2;
    waypoints.push_back(target_pose);  
    target_pose.position.z += 0.2;
    target_pose.position.y += 0.2;
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);  

    moveit_msgs::msg::RobotTrajectory traj;
    const double jump_th = 0.0;
    const double step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, step, jump_th, traj);
    if( fraction > 0.8 )
        move_group.execute(traj); 
    else 
        RCLCPP_ERROR(rclcpp::get_logger("moveit_executor"), "Trajectory calculation failed");

}


void CartesianPlanning::run() {
    auto node = rclcpp::Node::make_shared("CartesianPlan");
    _node = node;
    std::thread cartesian_plan_t( &CartesianPlanning::plan, this);
    rclcpp::spin(node);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  CartesianPlanning cp;
  cp.run();
}




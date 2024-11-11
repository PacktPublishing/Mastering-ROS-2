#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>



#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "boost/thread.hpp"

class PlanningWithObstacles : public rclcpp::Node
{
public:
    PlanningWithObstacles() : Node("cartesian_planning") {}
    void run();
    void plan();
    void setup_world();
private:
    rclcpp::Node::SharedPtr _node;
    moveit::planning_interface::MoveGroupInterface *_move_group; //(_node, "arm");

};

void PlanningWithObstacles::plan() {
    bool tf_found = false;    
    geometry_msgs::msg::TransformStamped t;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    //moveit::planning_interface::MoveGroupInterface move_group(_node, "arm");
    _move_group = new moveit::planning_interface::MoveGroupInterface(_node, "arm");
    
    std::string fromFrameRel = _move_group->getPlanningFrame().c_str();
    std::string toFrameRel =   _move_group->getEndEffectorLink().c_str();
    
    setup_world();

    sleep(3);
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
    target_pose.position.x += 0.3;
    _move_group->setPoseTarget(target_pose); 
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (_move_group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    _move_group->move(); 

}


void PlanningWithObstacles::run() {
    auto node = rclcpp::Node::make_shared("CartesianPlan");
    _node = node;
    boost::thread cartesian_plan_t( &PlanningWithObstacles::plan, this);
    rclcpp::spin(node);
}

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  PlanningWithObstacles cp;
  cp.run();
}


void PlanningWithObstacles::setup_world() {

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.header.frame_id = _move_group->getPlanningFrame();
    obstacle.id = "table";
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.3;
    geometry_msgs::msg::Pose bp;
    bp.orientation.w = 1.0;
    bp.position.x = 0.48;
    bp.position.y = 0.0;
    bp.position.z = 0.25;

    obstacle.primitives.push_back(primitive);
    obstacle.primitive_poses.push_back(bp);
    obstacle.operation = obstacle.ADD;
    planning_scene_interface.applyCollisionObject(obstacle);

    
    moveit_msgs::msg::CollisionObject grasping_object;
    grasping_object.id = "grasp";
    shape_msgs::msg::SolidPrimitive grasping_object_primitive;
    grasping_object_primitive.type = primitive.CYLINDER;
    grasping_object_primitive.dimensions.resize(2);
    grasping_object_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.1;
    grasping_object_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;
    grasping_object.header.frame_id = _move_group->getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.28;
    grasping_object.primitives.push_back(grasping_object_primitive);
    grasping_object.primitive_poses.push_back(grab_pose);
    grasping_object.operation = grasping_object.ADD;
    planning_scene_interface.applyCollisionObject(grasping_object);
    std::vector<std::string> connection_links;
    
    connection_links.push_back ( "panda_rightfinger" );
    connection_links.push_back ( "panda_leftfinger" );
    _move_group->attachObject(grasping_object.id, "hand", connection_links);


}
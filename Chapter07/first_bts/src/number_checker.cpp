#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include "std_msgs/msg/int32.hpp"
#include <random>

using namespace std::chrono_literals;

class PublishResult : public BT::StatefulActionNode {
  public:
    PublishResult(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
        node_ = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
    }
 
    BT::NodeStatus onStart() {
      std::string topic_name;
      getInput<std::string>("topic_name", topic_name );
      pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic_name, 1);
      return BT::NodeStatus::RUNNING;
    }
  
    BT::NodeStatus onRunning() {   
      int value;      
      getInput<int>("generated_number", value );
      std_msgs::msg::Int32 v;
      v.data = value;
      pub_->publish( v );
      return BT::NodeStatus::SUCCESS;
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("topic_name"),
        BT::InputPort<int>("generated_number")
      };
    }

    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;

};


class NumberChecker : public BT::StatefulActionNode {
  public:
    NumberChecker(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
    }
 
    BT::NodeStatus onStart() {
        getInput<int>("check_value", num_threshold_ );
        
        std::cout << "Check value: " << num_threshold_ << std::endl;

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {   
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(1, 100); // For an integer between 1 and 100
        int random_number = distrib(gen);

        std::cout << "Generated number: " << random_number << " - Threshold: " << num_threshold_ << " - ";
        if( random_number < num_threshold_) {
          setOutput("generated_number", random_number);
          std::cout << "Success! " << std::endl;
          return BT::NodeStatus::SUCCESS;
        }
        else {
          std::cout << "Failure! " << std::endl;
          setOutput("generated_number", -1);
          return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<int>("check_value"),
        BT::OutputPort<int>("generated_number")
      };
    }

    private:
        int num_threshold_;
};

 
 
class BTExecutor : public rclcpp::Node {
  public:
    BTExecutor()
    : Node("bt_executor") {
      first_ = true;
      timer_ = this->create_wall_timer( 0.5s, std::bind(&BTExecutor::tick_function, this));
      blackboard_ = BT::Blackboard::create();

    }

  private:

    void init_btree() {
      
        //blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        
        factory_.registerNodeType<NumberChecker>("CheckNumber1");
        factory_.registerNodeType<NumberChecker>("CheckNumber2");
        factory_.registerNodeType<NumberChecker>("CheckNumber3");
        factory_.registerNodeType<PublishResult>("PublishResult");

        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);

    }

    void tick_function() {
      if( first_ ) {
          init_btree();
          first_ = false;
      } 

      tree_.tickOnce();

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Blackboard::Ptr blackboard_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
    bool first_;


};



 
int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr BT_executor_node = std::make_shared<BTExecutor>();
  rclcpp::spin( BT_executor_node );
  rclcpp::shutdown();

  return 0;

}
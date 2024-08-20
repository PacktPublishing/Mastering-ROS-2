
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

class HelloNode : public BT::StatefulActionNode {
  public:
    HelloNode(const std::string & action_name, const BT::NodeConfig & conf) : BT::StatefulActionNode(action_name, conf) {    
    }
 
    BT::NodeStatus onStart() {
        getInput<std::string>("msg", _hello_msg );
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        std::cout << _hello_msg << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("msg")
      };
    }

    private:        
        std::string _hello_msg;
};

 
 
class BTExecutor : public rclcpp::Node {
  public:
    BTExecutor()
    : Node("bt_executor") {
        init_btree();
        timer_ = this->create_wall_timer( 0.5s, std::bind(&BTExecutor::tick_function, this));
    }

  private:

    void init_btree() {      
        factory_.registerNodeType<HelloNode>("HelloNode1");
        factory_.registerNodeType<HelloNode>("HelloNode2");
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        _tree = factory_.createTreeFromFile(tree_file);

    }

    void tick_function() {
        _tree.tickOnce();
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree _tree;
    BT::BehaviorTreeFactory factory_;
};

 
int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin( std::make_shared<BTExecutor>() );
  rclcpp::shutdown();
  return 0;
}
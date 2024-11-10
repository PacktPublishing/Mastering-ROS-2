#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include <bt_action_server/action/reach_location.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


using namespace std::chrono_literals;
using Action = bt_action_server::action::ReachLocation;

enum class ActionResult : uint8_t {  
    ActionNotCompleted,
    ActionFailed,
    ActionCancelled,
    ActionSucceded
};

class WaitForServer : public BT::StatefulActionNode {
  public:
    WaitForServer(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
        _node = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
    }
 
    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name );

        client_ = rclcpp_action::create_client<Action>(_node, server_name);
        return BT::NodeStatus::RUNNING;
    }
  
    BT::NodeStatus onRunning() {   
      
        if (!client_->action_server_is_ready()) {
            std::cout << "[Node WaitForServer]: Failure"<<std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else {
            std::cout << "[Node WaitForServer]: Success"<<std::endl;
            return BT::NodeStatus::SUCCESS;

        }
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("server_name")
      };
    }

    private:
      rclcpp::Node::SharedPtr _node;
      rclcpp_action::Client<Action>::SharedPtr client_;
};


class CallAction : public BT::StatefulActionNode {
  public:
    CallAction(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
        node2_ = rclcpp::Node::make_shared("action_client_node");
    }
 
    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name );
        getInput<float>("x", x_ );
        getInput<float>("y", y_ );
        getInput<float>("timeout", timeout_ );
        action_result_ = ActionResult::ActionNotCompleted; 
        server_called_ = false;

        client_ = rclcpp_action::create_client<Action>(node2_, server_name);
        return BT::NodeStatus::RUNNING;
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result) {
        
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                action_result_ = ActionResult::ActionSucceded;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                action_result_ = ActionResult::ActionFailed;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                action_result_ = ActionResult::ActionCancelled;
                break;
            default:
                break;
        }
    }
    BT::NodeStatus onRunning() {   

        if ( !server_called_ ) {
            server_called_ = true;
            auto goal = Action::Goal();
            goal.x = x_;
            goal.y = y_;
            goal.timeout = timeout_;
            auto send_goal_future = client_->async_send_goal(goal);
            
            if (rclcpp::spin_until_future_complete(node2_, send_goal_future) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                    std::cout << "Failed to send goal" <<std::endl;
                    return BT::NodeStatus::FAILURE;
            }
                    
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "Goal was rejected by server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            client_->async_get_result(goal_handle, std::bind(&CallAction::result_callback, this, std::placeholders::_1));
           

        }

        rclcpp::spin_some( node2_ );

        if( action_result_ ==  ActionResult::ActionSucceded ) {
            std::cout << "[Node CallAction]: Success" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if( action_result_ == ActionResult::ActionFailed ) {
            std::cout << "[Node CallAction]: Failure" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else if( action_result_ == ActionResult::ActionCancelled) {
            std::cout << "[Node CallAction]: Failure" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        
        std::cout << "[Node CallAction]: Waiting server execution" << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() { return; }

    static BT::PortsList providedPorts() {
      return {
        BT::InputPort<std::string>("server_name"),
        BT::InputPort<float>("x"),
        BT::InputPort<float>("y"),
        BT::InputPort<float>("timeout")
      };
    }

    private:
        rclcpp::Node::SharedPtr node2_;
        rclcpp_action::Client<Action>::SharedPtr client_;
        float x_, y_, timeout_;
        bool server_called_;
        ActionResult action_result_;
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
      
        blackboard_->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        factory_.registerNodeType<WaitForServer>("WaitForServer");
        factory_.registerNodeType<CallAction>("CallAction");
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
        
    }

    void tick_function() {
      if( first_) {
          init_btree();
          first_ = false;
      } 
      tree_.tickOnce();
    }
    
    BT::Tree tree_;
    BT::Blackboard::Ptr blackboard_;
    BT::BehaviorTreeFactory factory_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool first_;
};

 
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    rclcpp::spin( node );
    rclcpp::shutdown();
    return 0;
}
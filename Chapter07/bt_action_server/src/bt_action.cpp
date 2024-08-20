#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/blackboard.h"
#include "behaviortree_cpp/bt_factory.h"
#include <bt_action_server/action/reach_location.hpp>
#include <rclcpp_action/rclcpp_action.hpp>


using namespace std::chrono_literals;
using Action = bt_action_server::action::ReachLocation;

class WaitForServer : public BT::StatefulActionNode {
  public:
    WaitForServer(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
        _node = conf.blackboard->get<rclcpp::Node::SharedPtr>("node");
    }
 
    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name );

        _client = rclcpp_action::create_client<Action>(_node, server_name);
        return BT::NodeStatus::RUNNING;
    }
  
    BT::NodeStatus onRunning() {   
      
        if (!_client->action_server_is_ready()) {
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
      rclcpp_action::Client<Action>::SharedPtr _client;
};


class CallAction : public BT::StatefulActionNode {
  public:
    CallAction(const std::string & action_name, const BT::NodeConfig & conf) : 
      BT::StatefulActionNode(action_name, conf) {    
        _node2 = rclcpp::Node::make_shared("action_client_node");
    }
 
    BT::NodeStatus onStart() {
        std::string server_name;
        getInput<std::string>("server_name", server_name );
        getInput<float>("x", _x );
        getInput<float>("y", _y );
        getInput<float>("timeout", _timeout );
        _action_result = -1;
        _server_called = false;

        _client = rclcpp_action::create_client<Action>(_node2, server_name);
        return BT::NodeStatus::RUNNING;
    }
    void result_callback(const rclcpp_action::ClientGoalHandle<Action>::WrappedResult & result) {
        
        
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                _action_result = 2;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                _action_result = 1;
                break;
            case rclcpp_action::ResultCode::CANCELED:
                _action_result = 0;
                break;
            default:
                break;
        }
    }
    BT::NodeStatus onRunning() {   

        if ( !_server_called ) {
            _server_called = true;
            auto goal = Action::Goal();
            goal.x = _x;
            goal.y = _y;
            goal.timeout = _timeout;
            auto send_goal_future = _client->async_send_goal(goal);
            
            if (rclcpp::spin_until_future_complete(_node2, send_goal_future) !=
                rclcpp::FutureReturnCode::SUCCESS) {
                    std::cout << "Failed to send goal" <<std::endl;
                    return BT::NodeStatus::FAILURE;
            }
                    
            auto goal_handle = send_goal_future.get();
            if (!goal_handle) {
                std::cout << "Goal was rejected by server" << std::endl;
                return BT::NodeStatus::FAILURE;
            }
            _client->async_get_result(goal_handle, std::bind(&CallAction::result_callback, this, std::placeholders::_1));
           

        }

        rclcpp::spin_some( _node2 );

        if( _action_result == 2 ) {
            std::cout << "[Node CallAction]: Success" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if( _action_result == 1 ) {
            std::cout << "[Node CallAction]: Failure" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
        else if( _action_result == 0 ) {
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
        rclcpp::Node::SharedPtr _node2;
        rclcpp_action::Client<Action>::SharedPtr _client;
        float _x, _y, _timeout;
        bool _server_called;
        int _action_result;
};


class BTExecutor : public rclcpp::Node {
  public:
    BTExecutor()
    : Node("bt_executor") {
      _first = true;
      timer_ = this->create_wall_timer( 0.5s, std::bind(&BTExecutor::tick_function, this));
      _blackboard = BT::Blackboard::create();

    }

  private:

    void init_btree() {
      
        _blackboard->set<rclcpp::Node::SharedPtr>("node", this->shared_from_this());
        _factory.registerNodeType<WaitForServer>("WaitForServer");
        _factory.registerNodeType<CallAction>("CallAction");
        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        _tree = _factory.createTreeFromFile(tree_file, _blackboard);
        
    }

    void tick_function() {
      if( _first) {
          init_btree();
          _first = false;
      } 
      _tree.tickOnce();
    }
    
    BT::Tree _tree;
    BT::Blackboard::Ptr _blackboard;
    BT::BehaviorTreeFactory _factory;
    rclcpp::TimerBase::SharedPtr timer_;
    bool _first;
};

 
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    rclcpp::spin( node );
    rclcpp::shutdown();
    return 0;
}
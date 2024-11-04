#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define J_SIZE 2

namespace sine_controller {
  class SineController : public controller_interface::ControllerInterface
  {
  private:  
    rclcpp::Duration dt_;
    template<typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;
    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_;
    std::vector<std::vector<std::string>> state_interface_names_;
    
    float initial_joint_position_[J_SIZE];
    float desired_joint_positions_[J_SIZE];
    float amplitude_[J_SIZE];
    float frequency_[J_SIZE];
    double t_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sine_param_sub_;

  public:
    SineController() : controller_interface::ControllerInterface(), dt_(0, 0) {}

    void sine_param_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)  {
      
      if( msg->data.size() != 4 ) {        
        RCLCPP_ERROR(this->get_node()->get_logger(), "Wrong number of sine parameters");
        return;
      }
      amplitude_[0] = msg->data[0];
      amplitude_[1] = msg->data[2];
      frequency_[0] = msg->data[1];
      frequency_[1] = msg->data[3];

    }

    controller_interface::InterfaceConfiguration state_interface_configuration() const
    {
      std::vector<std::string> state_interfaces_config_names;
      state_interfaces_config_names.push_back("base_joint/position");
      state_interfaces_config_names.push_back("base_joint/velocity");
      state_interfaces_config_names.push_back("link1_link2/position");
      state_interfaces_config_names.push_back("link1_link2/velocity");
   
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
    }

    
    controller_interface::InterfaceConfiguration command_interface_configuration() const
    {
      std::vector<std::string> command_interfaces_config_names;
      command_interfaces_config_names.push_back("base_joint/position");
      command_interfaces_config_names.push_back("link1_link2/position");      
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names};
    }

    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      
      sine_param_sub_ =   get_node()->create_subscription<std_msgs::msg::Float32MultiArray>("/sine_param", 10, 
            std::bind(&SineController::sine_param_cb, this, std::placeholders::_1));

      dt_ = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / get_update_rate()));

      amplitude_[0] = 0.0;
      amplitude_[1] = 0.0;
      frequency_[0] = 0.0;
      frequency_[1] = 0.0;

      command_interfaces_.reserve     (J_SIZE); 
      state_interfaces_.reserve       (2 * J_SIZE);
      joint_state_interfaces_.resize  (J_SIZE);
      state_interface_names_.resize   (J_SIZE);
      
      


      std::vector<std::string> joint_name = {"base_joint", "link1_link2"};
      for(int i = 0; i < J_SIZE; i++) {
        state_interface_names_[i].resize(2);
        state_interface_names_[i][0] = joint_name[i] + "/position";
        state_interface_names_[i][1] = joint_name[i] + "/velocity";  
      }

  
      RCLCPP_INFO(get_node()->get_logger(), "configure successful");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
      for(int i = 0; i < 2; i++) 
        controller_interface::get_ordered_interfaces( state_interfaces_, state_interface_names_[i], std::string(""),joint_state_interfaces_[i]);
      
      initial_joint_position_[0] = joint_state_interfaces_[0][0].get().get_value();
      initial_joint_position_[1] = joint_state_interfaces_[1][0].get().get_value();
      RCLCPP_INFO(get_node()->get_logger(), "Activate successful");
      t_ = 0.0;

      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_init() {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      return controller_interface::CallbackReturn::SUCCESS;
    }
    
    controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
      t_ += dt_.seconds();
      for (int i = 0; i < 2; i++) {        
        desired_joint_positions_[i] = initial_joint_position_[i] + (amplitude_[i] * std::sin(2 * M_PI * frequency_[i] * t_));
        command_interfaces_[i].set_value(desired_joint_positions_[i]);
      }  
      return controller_interface::return_type::OK;
    }
  };
} // namespace sine_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface)

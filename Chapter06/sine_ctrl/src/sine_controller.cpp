#include "rclcpp/rclcpp.hpp" // ROS 2 core library
#include "controller_interface/controller_interface.hpp" // Base class for ROS 2 controllers
#include "controller_interface/helpers.hpp" // Helper functions for controller interfaces
#include "std_msgs/msg/float32_multi_array.hpp" // Message type for receiving sine parameters

#define J_SIZE 2 // Define joint size for two joints in the controller

namespace sine_controller {
  // Custom controller class implementing a sine wave-based controller for a 2-joint system
  class SineController : public controller_interface::ControllerInterface
  {
  private:
    rclcpp::Duration dt_; // Duration between updates, calculated from update rate

    // Template alias for managing references to state interfaces
    template<typename T>
    using InterfaceReferences = std::vector<std::vector<std::reference_wrapper<T>>>;

    InterfaceReferences<hardware_interface::LoanedStateInterface> joint_state_interfaces_; // References to state interfaces
    std::vector<std::vector<std::string>> state_interface_names_; // Stores names of state interfaces for each joint

    float initial_joint_position_[J_SIZE]; // Initial positions for each joint
    float desired_joint_positions_[J_SIZE]; // Desired positions based on sine wave calculations
    float amplitude_[J_SIZE]; // Amplitudes for sine wave generation for each joint
    float frequency_[J_SIZE]; // Frequencies for sine wave generation for each joint
    double t_; // Time accumulator for sine wave calculation

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sine_param_sub_; // Subscription to sine parameter messages

  public:
    // Constructor initializes the base controller and time increment duration
    SineController() : controller_interface::ControllerInterface(), dt_(0, 0) {}

    // Callback for updating sine parameters received via ROS topic
    void sine_param_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)  {
      if (msg->data.size() != 4) {        
        RCLCPP_ERROR(this->get_node()->get_logger(), "Wrong number of sine parameters");
        return;
      }
      // Assign received amplitudes and frequencies to respective joints
      amplitude_[0] = msg->data[0];
      amplitude_[1] = msg->data[2];
      frequency_[0] = msg->data[1];
      frequency_[1] = msg->data[3];
    }

    // Configure state interfaces, specifying which states to read for each joint
    controller_interface::InterfaceConfiguration state_interface_configuration() const
    {
      std::vector<std::string> state_interfaces_config_names = {
        "base_joint/position",
        "base_joint/velocity",
        "link1_link2/position",
        "link1_link2/velocity"
      };
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, state_interfaces_config_names};
    }

    // Configure command interfaces, specifying which commands to control for each joint
    controller_interface::InterfaceConfiguration command_interface_configuration() const
    {
      std::vector<std::string> command_interfaces_config_names = {
        "base_joint/position",
        "link1_link2/position"
      };
      return {
          controller_interface::interface_configuration_type::INDIVIDUAL, command_interfaces_config_names};
    }

    // Lifecycle callback to configure controller parameters and resources
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
      // Set up subscription to sine parameters
      sine_param_sub_ = get_node()->create_subscription<std_msgs::msg::Float32MultiArray>(
          "/sine_param", 10, std::bind(&SineController::sine_param_cb, this, std::placeholders::_1));
      
      // Set time increment between updates based on update rate
      dt_ = rclcpp::Duration(std::chrono::duration<double, std::milli>(1e3 / get_update_rate()));

      // Initialize amplitude and frequency arrays to zero
      amplitude_[0] = amplitude_[1] = 0.0;
      frequency_[0] = frequency_[1] = 0.0;

      // Reserve space in vectors for efficiency and initialize state interface references
      command_interfaces_.reserve(J_SIZE);
      state_interfaces_.reserve(2 * J_SIZE);
      joint_state_interfaces_.resize(J_SIZE);
      state_interface_names_.resize(J_SIZE);

      // Assign interface names for each joint's position and velocity
      std::vector<std::string> joint_name = {"base_joint", "link1_link2"};
      for (int i = 0; i < J_SIZE; i++) {
        state_interface_names_[i] = {joint_name[i] + "/position", joint_name[i] + "/velocity"};
      }

      RCLCPP_INFO(get_node()->get_logger(), "Configuration successful");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // Lifecycle callback to activate the controller, initializing initial joint positions
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &)
    {
      // Initialize references to the ordered state interfaces and get initial positions
      for (int i = 0; i < 2; i++)
        controller_interface::get_ordered_interfaces(state_interfaces_, state_interface_names_[i], "", joint_state_interfaces_[i]);
      
      initial_joint_position_[0] = joint_state_interfaces_[0][0].get().get_value();
      initial_joint_position_[1] = joint_state_interfaces_[1][0].get().get_value();
      RCLCPP_INFO(get_node()->get_logger(), "Activation successful");
      t_ = 0.0;

      return controller_interface::CallbackReturn::SUCCESS;
    }

    // Callback for initialization state
    controller_interface::CallbackReturn on_init() {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // Callback for deactivation state
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    // Main update loop, computes sine wave for joint positions and applies them to command interfaces
    controller_interface::return_type update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
      t_ += dt_.seconds(); // Increment time for sine calculations
      for (int i = 0; i < 2; i++) {
        // Calculate desired joint position as a sine wave based on initial position, amplitude, and frequency
        desired_joint_positions_[i] = initial_joint_position_[i] + (amplitude_[i] * std::sin(2 * M_PI * frequency_[i] * t_));
        command_interfaces_[i].set_value(desired_joint_positions_[i]); // Apply position command
      }
      return controller_interface::return_type::OK;
    }
  };
} // namespace sine_controller

#include "pluginlib/class_list_macros.hpp" // Plugin library for dynamically loading the controller
PLUGINLIB_EXPORT_CLASS(sine_controller::SineController, controller_interface::ControllerInterface) // Export for use as plugin


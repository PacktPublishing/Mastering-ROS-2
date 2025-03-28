//#ifndef rqt_image_view__ImageView_H
//#define rqt_image_view__ImageView_H


#include "geometry_msgs/msg/twist.hpp"
#include "ui_cmd_vel_ui.h"
#include <rqt_gui_cpp/plugin.h>

namespace cmd_vel {

class CmdVel
  : public rqt_gui_cpp::Plugin
    {
        Q_OBJECT  // Add this macro
        public:
            CmdVel();
            //~CmdVel() override = default;
            virtual void initPlugin(qt_gui_cpp::PluginContext& context);
            virtual void shutdownPlugin();
            void onTextChanged();
            void onVelChanged();
            void onEnableButton(int state);
            void pubCmd();            
            void onStopButton();
            void onForwardButton();
            void onBackwardButton();
            void onLeftButton();
            void onRightButton();
            void onUpButton();
            void onDownButton();
        protected:
            Ui::CmdVelWidget ui_;
            QWidget* widget_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_; // ROS 2 Publisher
            rclcpp::Node::SharedPtr ros2_node_;         // ROS 2 Node
            geometry_msgs::msg::Twist cmd_vel_msg_;            
            bool enable_streaming_;
            double des_vel_;
    };
}
//#endif // rqt_image_view__ImageView_H

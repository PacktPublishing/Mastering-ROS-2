#include "cmd_vel/cmd_vel.h" 
#include <pluginlib/class_list_macros.hpp> 
#include <rclcpp/rclcpp.hpp> 
#include <sstream>  

namespace cmd_vel {
    CmdVel::CmdVel() : rqt_gui_cpp::Plugin(),   // Call base class constructor
        widget_(0),
        enable_streaming_(false) {
        setObjectName("CmdVel");        
    }
    
    void CmdVel::initPlugin(qt_gui_cpp::PluginContext& context) {
        widget_ = new QWidget();
        ui_.setupUi(widget_);

        if (context.serialNumber() > 1) {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);

        // Create ROS 2 node
        ros2_node_ = rclcpp::Node::make_shared("custom_cmd_vel_plugin");        
        connect(ui_.topicName, &QLineEdit::returnPressed, this, &CmdVel::onTextChanged);
        connect(ui_.desVel, &QLineEdit::returnPressed, this, &CmdVel::onVelChanged);
        connect(ui_.enableButton, &QCheckBox::stateChanged, this, &CmdVel::onEnableButton);        
        connect(ui_.stopButton, &QPushButton::clicked, this, &CmdVel::onStopButton);  
        connect(ui_.forwardButton, &QPushButton::clicked, this, &CmdVel::onForwardButton);  
        connect(ui_.backwardButton, &QPushButton::clicked, this, &CmdVel::onBackwardButton);  
        connect(ui_.leftButton, &QPushButton::clicked, this, &CmdVel::onLeftButton);  
        connect(ui_.rightButton, &QPushButton::clicked, this, &CmdVel::onRightButton);  
        connect(ui_.upButton, &QPushButton::clicked, this, &CmdVel::onUpButton);  
        connect(ui_.downButton, &QPushButton::clicked, this, &CmdVel::onDownButton);  
        des_vel_ = 0.1;

    }    

    void CmdVel::shutdownPlugin() {}

    void CmdVel::onVelChanged() {

        double str2double;
        std::istringstream iss( ui_.desVel->text().toStdString() );
        iss >> str2double;

        if (!(iss.fail() || !iss.eof()) ) 
            des_vel_ = str2double;
        
    }
    void CmdVel::onTextChanged() {        
        std::string topicNameValue = ui_.topicName->text().toStdString();
        if( topicNameValue != "" )
            cmd_vel_publisher_ = ros2_node_->create_publisher<geometry_msgs::msg::Twist>(topicNameValue, 10);
    }
    
    void CmdVel::onEnableButton(int state) {
        if( state == Qt::Checked ) {
            enable_streaming_ = true;
        }
        else {
            enable_streaming_ = false;
        }
    }

    void CmdVel::onStopButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }   

    void CmdVel::onForwardButton() {

        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.linear.x = des_vel_;
        if( enable_streaming_ )
            if( cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }       

    void CmdVel::onBackwardButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.linear.x = -des_vel_;
        if( enable_streaming_ && cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }     

    void CmdVel::onLeftButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.angular.z = des_vel_;
        if( enable_streaming_ )
            if( cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }  
    void CmdVel::onRightButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.angular.z = -des_vel_;
        if( enable_streaming_ )
            if( cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }        
    void CmdVel::onUpButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.linear.z = des_vel_;
        if( enable_streaming_ )
            if( cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }  
    void CmdVel::onDownButton() {
        cmd_vel_msg_ = geometry_msgs::msg::Twist();
        cmd_vel_msg_.linear.z = -des_vel_;
        if( enable_streaming_ )
            if( cmd_vel_publisher_ ) cmd_vel_publisher_->publish( cmd_vel_msg_ );
    }  
    
}

PLUGINLIB_EXPORT_CLASS(cmd_vel::CmdVel, rqt_gui_cpp::Plugin)

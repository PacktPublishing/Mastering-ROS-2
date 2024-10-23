#include "px4_ctrl.h"

using namespace std;

void Px4Control::arm() {
	RCLCPP_INFO(this->get_logger(), "Arming");
	send_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0);
}

void Px4Control::disarm() {
	RCLCPP_INFO(this->get_logger(), "Disarming");
	send_cmd(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0);
}
void Px4Control::uav_pose_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    _curr_x = msg->x;
    _curr_y = msg->y;
    _curr_z = msg->z;
}

void Px4Control::srv_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
        auto reply = future.get()->reply;
        uint8_t service_result_ = reply.result;
        _action_done = true;
    } 
    else {
        RCLCPP_INFO(this->get_logger(), "Service In-Progress...");  
    }
}

void Px4Control::publish_offboard_ctrl_mode() {
	
    rclcpp::Rate loop_rate(1);
    
    while( rclcpp::ok() ) {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    	_offboard_ctrl_mode_publisher->publish(msg);
        loop_rate.sleep();
    }
}

void Px4Control::send_cmd(uint16_t command, float param1, float param2) {
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand cmd{};
	cmd.param1 = param1;
	cmd.param2 = param2;
	cmd.command = command;
	cmd.target_system = 1;
	cmd.target_component = 1;
	cmd.source_system = 1;
	cmd.source_component = 1;
	cmd.from_external = true;
	cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = cmd;

	auto result = _cmd_client->async_send_request(request, std::bind(&Px4Control::srv_callback, this,
                           std::placeholders::_1));
}

struct Point3D {
    double x, y, z;
};

// Function to calculate the Euclidean distance between two 3D points
double calculateDistance(const Point3D& start, const Point3D& end) {
    return std::sqrt(std::pow(end.x - start.x, 2) +
                     std::pow(end.y - start.y, 2) +
                     std::pow(end.z - start.z, 2));
}

std::vector<Point3D> planTrajectory(const Point3D& start, const Point3D& end, double v_max, double time_step) {
    std::vector<Point3D> waypoints;
    
    // Calculate the total distance between the start and end points
    double total_distance = calculateDistance(start, end);

    // Calculate the total time required to reach the end point
    double total_time = total_distance / v_max;

    // Calculate the number of time steps needed
    int num_steps = std::ceil(total_time / time_step);

    // Calculate the velocity components for each axis
    double v_x = (end.x - start.x) / total_time;
    double v_y = (end.y - start.y) / total_time;
    double v_z = (end.z - start.z) / total_time;

    // Generate waypoints for each time step
    for (int i = 0; i <= num_steps; ++i) {
        double t = i * time_step;  // Current time

        // Calculate the position at time t
        Point3D waypoint;
        waypoint.x = start.x + v_x * t;
        waypoint.y = start.y + v_y * t;
        waypoint.z = start.z + v_z * t;

        waypoints.push_back(waypoint);
    }

    return waypoints;
}

void Px4Control::menu() {

    boost::thread publish_offboard_ctrl_mode_t( &Px4Control::publish_offboard_ctrl_mode, this );

    string cmd = "";
    while (rclcpp::ok() && cmd != "exit")  {
        _action_done = false;
        cout << "Insert the desired command" << endl;
        cout << "arm - to arm the drone" << endl;
        cout << "disarm - to disarm the drone" << endl;
        cout << "takeoff - to take the drone" << endl;
        cout << "move - specify a target point to reach" << endl;
        cout << endl;

        getline( cin, cmd );

        if( cmd == "arm") {
            arm();
            while( !_action_done) usleep(0.1*1e6);
        }
        else if ( cmd == "disarm" ) {
            disarm();
            while( !_action_done) usleep(0.1*1e6);
        }
        else if ( cmd == "takeoff") {
            send_cmd(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
            sleep(1);
            px4_msgs::msg::TrajectorySetpoint msg{};
            msg.position = {_curr_x, _curr_y, _curr_z-5.0};
            msg.yaw = 0.0; 
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            _traj_cmd_pub->publish(msg);
        }
        else if( cmd == "move") {
            float x, y, z;
            std::string input;
            std::cout << "Insert the destination point (x, y, z): ";
            std::getline(std::cin, input);  // Read entire line
            std::istringstream stream(input);
            stream >> x >> y >> z;

            Point3D start = {_curr_x, _curr_y, _curr_z};  
            Point3D end = {x, y, z}; 
            double v_max = 0.5; // 2 meters per second
            double time_step = 1.0/10.0; 
            std::vector<Point3D> trajectory = planTrajectory(start, end, v_max, time_step);
            px4_msgs::msg::TrajectorySetpoint msg{};
            rclcpp::Rate loop_rate(10);

            for (const auto& point : trajectory) {
                msg.position = {point.x, point.y, point.z};
                msg.yaw = 0.0; 
                msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
                _traj_cmd_pub->publish(msg);
                loop_rate.sleep();
            }
        }
    }
}

void Px4Control::run() {
    boost::thread menu_t( &Px4Control::menu, this);
    rclcpp::spin(shared_from_this());
}


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	auto node = std::make_shared<Px4Control>();    
    node->run();
    rclcpp::shutdown();
	return 0;
}

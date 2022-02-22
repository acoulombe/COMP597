#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <string>

// Global Variable Declarations
geometry_msgs::Twist g_vel;
bool g_newDataAvailable;

void TeleopCallBack (const std_msgs::String& msg){
	ROS_INFO("husky_teleop_controller subscriber: %s", msg.data.c_str());

	// Make data holder for converted data
	char* p_key = new char [msg.data.length()+1];
	std::strcpy(p_key, msg.data.c_str());

	if(
		p_key[0] == 'i' || p_key[0] == 'I' 
	){
		// Set Twist for going straight
		g_vel.linear.x = 1;
		g_vel.linear.y = 0;
		g_vel.linear.z = 0;

		g_vel.angular.x = 0;
		g_vel.angular.y = 0;
		g_vel.angular.z = 0;

		// Tell publisher to send data to velocity controller
		g_newDataAvailable = true;	
	}
	else if(
		p_key[0] == 'o' || p_key[0] == 'O'
	){
		// Set Twist for turning right
		g_vel.linear.x = 1;
		g_vel.linear.y = 0;
		g_vel.linear.z = 0;

		g_vel.angular.x = 0;
		g_vel.angular.y = 0;
		g_vel.angular.z = -1;

		// Tell publisher to send data to velocity controller
		g_newDataAvailable = true;	
	}
	else if(
		p_key[0] == 'u' || p_key[0] == 'U'
	){
		// Set Twist for turning left
		g_vel.linear.x = 1;
		g_vel.linear.y = 0;
		g_vel.linear.z = 0;

		g_vel.angular.x = 0;
		g_vel.angular.y = 0;
		g_vel.angular.z = 1;

		// Tell publisher to send data to velocity controller
		g_newDataAvailable = true;	
	}else{
		// Unknown key to process, do nothing
	}

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "husky_teleop_controller");
	ros::NodeHandle nh;

	// Init teleop cmd subscriber
	ros::Subscriber teleop_subscriber = nh.subscribe("/teleop/cmd", 1, TeleopCallBack);
	std_msgs::String teleop_msgs;

	// Init twist publisher
	ros::Publisher twist_publisher = nh.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);

	// Teleop Controller Process
	while(true){	

		if(g_newDataAvailable){
			// New Teleoperation data to send to husky velocity controller
			twist_publisher.publish(g_vel);
			g_newDataAvailable = false;
		}

		ros::spinOnce();
	}

	return 0;
}

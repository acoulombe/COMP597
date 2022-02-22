#include <ros/ros.h>
#include <cubic_polynomial_planner/target_pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <string.h>

// Global Variables
ros::Subscriber robot_state_subscriber;
double g_Xcurr = 0, g_Ycurr = 0, g_Zcurr = 0;
double g_Xstart = 0, g_Ystart = 0, g_Zstart = 0;
double g_Xtarget = 0, g_Ytarget = 0, g_Ztarget = 0;
double g_timeStart = 0, g_T = 1;

/**
 * ROS Service for getting the required information and planning the cubic polynomial movement
 */
bool cubicPolynomialPlannerService(
	cubic_polynomial_planner::target_pose::Request &req, 
	cubic_polynomial_planner::target_pose::Response &res
){
	ROS_INFO("New Target Position Requested: x=%f, y=%f, z=%f in T=%f", req.x, req.y, req.z, req.T);
	// Save information to produce trajectory
	g_timeStart = ros::Time::now().toSec();
	g_Xtarget = req.x;
	g_Ytarget = req.y;
	g_Ztarget = req.z;
	g_Xstart = g_Xcurr;
	g_Ystart = g_Ycurr;
	g_Zstart = g_Zcurr;
	g_T = req.T;

	res.success = true;

	return true;
}

/**
 * ROS Subscriber Callback to read robot state and save to global variables
 */ 
void robotStateCallBack(const gazebo_msgs::ModelStates& msg){
	geometry_msgs::Pose robot_pose = msg.pose[1];
	g_Xcurr = robot_pose.position.x;
	g_Ycurr = robot_pose.position.y;
	g_Zcurr = robot_pose.position.z;
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "cubic_polynomial_planner_node");
	ros::NodeHandle nh;

    // Initialise ROS Server for target_pose
	ros::ServiceServer motion_planner_service = nh.advertiseService("/cubic_polynomial_planner_node/move_robot", cubicPolynomialPlannerService); 

	// Init robot state subscriber
	robot_state_subscriber = nh.subscribe("/gazebo/model_states", 1, robotStateCallBack);

	// Init robot pose publisher
	ros::Publisher pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/firefly/command/pose", 1);

	// Set Control loop rate
	ros::Rate loopRate(10);

	// Publish Target Pose based on computed cubic polynomial trajectory
	double current_time = 0, dt = 0;
	double coeff = 0;
	geometry_msgs::PoseStamped next_pose;
	while(ros::ok()){
		current_time = ros::Time::now().toSec();
		dt = current_time - g_timeStart;
		coeff = 3*pow(dt/g_T, 2) - 2*pow(dt/g_T, 3);

		if(dt > g_T){
			next_pose.pose.position.x = g_Xtarget;
			next_pose.pose.position.y = g_Ytarget;
			next_pose.pose.position.z = g_Ztarget;
		}else{
			next_pose.pose.position.x = g_Xstart + coeff *(g_Xtarget - g_Xstart);
			next_pose.pose.position.y = g_Ystart + coeff *(g_Ytarget - g_Ystart);
			next_pose.pose.position.z = g_Zstart + coeff *(g_Ztarget - g_Zstart);
		}
		next_pose.header.stamp = ros::Time::now();
		next_pose.header.frame_id = "Next Pose";

		pose_publisher.publish(next_pose);

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}
// Pinocchio Includes
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
#include <inverse_kinematic_controller/target_pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// Global Variables
ros::Subscriber robot_state_subscriber;

// Robot State Feedback
const int JOINT_ID = 7;
Eigen::VectorXd g_jointPos      = Eigen::VectorXd::Zero(JOINT_ID, 1);
Eigen::VectorXd g_jointVel      = Eigen::VectorXd::Zero(JOINT_ID, 1);
Eigen::VectorXd g_jointEffort   = Eigen::VectorXd::Zero(JOINT_ID, 1);

// Task Space Target and Current states
Eigen::VectorXd g_currXYZ   = Eigen::VectorXd::Zero(3, 1);
Eigen::VectorXd g_startXYZ  = Eigen::VectorXd::Zero(3, 1);
Eigen::VectorXd g_desXYZ    = Eigen::VectorXd::Zero(3, 1);
double g_timeStart = 0, g_T = -1;

/**
 * ROS Service for getting the required information and planning the cubic polynomial movement
 */
bool cubicPolynomialPlannerService(
	inverse_kinematic_controller::target_pose::Request &req, 
	inverse_kinematic_controller::target_pose::Response &res
){
	ROS_INFO("New Target Position Requested: x=%f, y=%f, z=%f in T=%f", req.x, req.y, req.z, req.T);
	// Save information to produce trajectory
	g_timeStart = ros::Time::now().toSec();
	g_desXYZ(0) = req.x;
	g_desXYZ(1) = req.y;
	g_desXYZ(2) = req.z;
	g_startXYZ = g_currXYZ;
	g_T = req.T;

	res.success = true;

	return true;
}

/**
 * ROS Subscriber Callback to read robot state and save to global variables
 */ 
void robotStateCallBack(const sensor_msgs::JointState& msg){
	
    for(int i=0; i < JOINT_ID; i++){
        g_jointPos(i) = msg.position[i];
        g_jointVel(i) = msg.velocity[i];
        g_jointEffort(i) = msg.effort[i];
    }
}


int main(int argc, char** argv) {

    // Init ROS Relationships
	ros::init(argc, argv, "inverse_kinematic_controller");
	ros::NodeHandle nh;

    // Initialise ROS Server for target_pose
	ros::ServiceServer motion_planner_service = nh.advertiseService("/cubic_polynomial_planner_node/move_robot", cubicPolynomialPlannerService); 

	// Init robot state subscriber
	robot_state_subscriber = nh.subscribe("/j2n7s300/joint_states", 1, robotStateCallBack);

	// Init robot pose publisher
	ros::Publisher q_publisher = nh.advertise<std_msgs::Float64MultiArray>("/j2n7s300/joint_group_position_controller/command", 1);

	// Set Control loop rate
    const int fCycle_Hz = 1000;
    double dtCycle_S = 1.0f/fCycle_Hz;
	ros::Rate loopRate(fCycle_Hz);

    // Get Robot Model
    std::string urdf_file_name = ros::package::getPath("inverse_kinematic_controller")  + "/urdf/no-gripper.urdf";

    pinocchio::Model model;
	pinocchio::urdf::buildModel(urdf_file_name, model, false);				// read the URDF file
    pinocchio::Data data(model);

	// Publish Target Joint Posiion based on computed cubic polynomial trajectory
	double current_time = 0, dt = 0;
	double coeff = 0;
    Eigen::VectorXd nextQ = Eigen::VectorXd::Zero(JOINT_ID, 1);
    // From Launch file
    nextQ[1] = 2.9;
    nextQ[3] = 1.3;
    nextQ[4] = -2.07;
    nextQ[5] = 1.4;
    
    Eigen::VectorXd qVel = Eigen::VectorXd::Zero(JOINT_ID, 1);
    Eigen::VectorXd nextXYZ = Eigen::VectorXd::Zero(3, 1);
    Eigen::VectorXd velXYZ = Eigen::VectorXd::Zero(3, 1);

    // Get Default Pose
    pinocchio::forwardKinematics(model, data, nextQ, qVel);

    pinocchio::SE3 currPose = data.oMi[JOINT_ID];   // end-effector pose
    g_currXYZ = currPose.translation();

    // Published data holder
	std_msgs::Float64MultiArray qMsg;
    qMsg.data.resize(JOINT_ID);

    // Publisher and Control cycle
	while(ros::ok()){
        // Compute cubic polynomial coefficient
		current_time = ros::Time::now().toSec();
		dt = current_time - g_timeStart;

        // Compute next pose
		if(dt <= g_T){
        	// Use cubic polynomial to move robot pose
            coeff = 3*pow(dt/g_T, 2) - 2*pow(dt/g_T, 3);
            nextXYZ = g_startXYZ + coeff *(g_desXYZ - g_startXYZ);
	
            // Get Current Pose
            pinocchio::forwardKinematics(model, data, g_jointPos, g_jointVel);
            pinocchio::SE3 currPose = data.oMi[JOINT_ID];   // end-effector pose
            g_currXYZ = currPose.translation(); 

            // Compute Jacobian
            Eigen::MatrixXd jacobian_local_world = Eigen::MatrixXd::Zero(6,7) ;
            pinocchio::computeAllTerms(model, data, g_jointPos, g_jointVel) ;
            pinocchio::getJointJacobian(model, data, JOINT_ID, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_local_world) ;
            Eigen::MatrixXd Jv = jacobian_local_world.topRows(3);
            Eigen::MatrixXd pinvJv = Jv.completeOrthogonalDecomposition().pseudoInverse();

            // Compute Nullspace Projection
            Eigen::MatrixXd nullspace = Eigen::MatrixXd::Identity(JOINT_ID, JOINT_ID) - pinvJv * Jv;

            // Compute current task velocity
            velXYZ = (nextXYZ - g_currXYZ)/dtCycle_S;
            qVel = pinvJv * velXYZ + nullspace * -g_jointPos; // Take home configuration as all joints at zero

            // Compute next joint positions
            nextQ = g_jointPos + qVel * dtCycle_S;
        }

        // Publish Joint Positions to Controller
        for(int i = 0; i < JOINT_ID; i++){
            qMsg.data[i] = nextQ[i];
        }

		q_publisher.publish(qMsg);

		ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

#include <inverse_dynamics_controller/robot.hpp>
#include <inverse_dynamics_controller/target_pose.h>
#include <inverse_dynamics_controller/potential_field_planner.hpp>
#include <inverse_dynamics_controller/inverse_dynamics_controller.hpp>

// ROS includes
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

// Global Variables
ros::Subscriber robot_state_subscriber;
int g_numJoints;
int g_numEffortJoints;
int g_endEffectorJointID;
bool g_useNullspace;
Eigen::VectorXd g_initJointConfig;
Eigen::VectorXd g_fingerTargetConfig;
Eigen::VectorXd g_nullspaceTargetConfig;
Eigen::VectorXd g_torqueLimits;

// Controller Parameters
Eigen::MatrixXd g_Kattract;
Eigen::MatrixXd g_taskStiff;
Eigen::MatrixXd g_taskDamp;
Eigen::MatrixXd g_jointStiff;
Eigen::MatrixXd g_jointDamp;
Eigen::MatrixXd g_nullspaceAttract;

// Planner Parameters
PotentialFieldPlanner g_VPF;

// Robot State Feedback
Eigen::VectorXd g_jointPos;
Eigen::VectorXd g_jointVel;
Eigen::VectorXd g_jointEffort;

// Task Space Target and Current states
Eigen::VectorXd g_currXYZ = Eigen::VectorXd::Zero(3, 1);
Eigen::VectorXd g_desXYZ = Eigen::VectorXd::Zero(3, 1);

/**
 * ROS Service for getting the required information and planning the cubic polynomial movement
 */
bool PlannerService(
	inverse_dynamics_controller::target_pose::Request &req, 
	inverse_dynamics_controller::target_pose::Response &res
){
	ROS_INFO("New Target Position Requested: x=%f, y=%f, z=%f", req.x, req.y, req.z);
	// Save information to produce trajectory
	g_desXYZ(0) = req.x;
	g_desXYZ(1) = req.y;
	g_desXYZ(2) = req.z;

    g_VPF.setGoalPosition(g_desXYZ);

	res.success = true;

	return true;
}

/**
 * ROS Subscriber Callback to read robot state and save to global variables
 */ 
void robotStateCallBack(const sensor_msgs::JointState& msg){
	
    for(int i=0; i < g_numJoints; i++){
        g_jointPos(i) = msg.position[i];
        g_jointVel(i) = msg.velocity[i];
        g_jointEffort(i) = msg.effort[i];
    }
}


int main(int argc, char** argv) {

    // Init ROS Relationships
	ros::init(argc, argv, "inverse_dynamics_controller");
	ros::NodeHandle nh;

    // Load rosparameters
    std::string torque_publisher_topic;
    nh.getParam("torque_publisher_topic", torque_publisher_topic);
    std::cout << "torque_publisher_topic: " << torque_publisher_topic << std::endl;

    std::string joint_subscriber_topic;
    nh.getParam("joint_subscriber_topic", joint_subscriber_topic);
    std::cout << "joint_subscriber_topic: " << joint_subscriber_topic << std::endl;

    std::string finger_publisher_topic;
    nh.getParam("finger_publisher_topic", finger_publisher_topic);

    std::string urdf_file_name;
    nh.getParam("urdf_file", urdf_file_name);
    std::cout << "urdf_file: " << urdf_file_name << std::endl;

    nh.getParam("use_nullspace", g_useNullspace);
    std::cout << "use_nullspace: " << g_useNullspace << std::endl;
    nh.getParam("num_joints", g_numJoints);
    std::cout << "num_joints: " << g_numJoints << std::endl;
    nh.getParam("num_effort_joints", g_numEffortJoints);
    std::cout << "num_effort_joints: " << g_numEffortJoints << std::endl;
    nh.getParam("end_effector_joint_id", g_endEffectorJointID);
    
    std::vector<double> tmpV1;
    nh.getParam("robot/init_joint_config", tmpV1);
    g_initJointConfig = Eigen::VectorXd::Zero(tmpV1.size(), 1);
    for(int i=0; i < tmpV1.size(); i++){
        g_initJointConfig[i] = tmpV1[i];
    }
    std::cout << "robot/init_joint_config: " << g_initJointConfig << std::endl;
    
    std::vector<double> tmpV2;
    nh.getParam("robot/finger_target_config", tmpV2);
    g_fingerTargetConfig = Eigen::VectorXd::Zero(tmpV2.size(), 1);
    for(int i=0; i < tmpV2.size(); i++){
        g_fingerTargetConfig[i] = tmpV2[i];
    }
    std::cout << "robot/finger_target_config: " << g_fingerTargetConfig << std::endl;

    std::vector<double> tmpV3;
    nh.getParam("robot/nullspace_target_config", tmpV3);
    g_nullspaceTargetConfig = Eigen::VectorXd::Zero(tmpV3.size(), 1);
    for(int i=0; i < tmpV3.size(); i++){
        g_nullspaceTargetConfig[i] = tmpV3[i];
    }
    std::cout << "robot/nullspace_target_config: "<< g_nullspaceTargetConfig << std::endl;

    std::vector<double> tmpV4;
    nh.getParam("planner/attractive_gain", tmpV4);
    g_Kattract = Eigen::MatrixXd::Zero(3, 3);
    for(int i=0; i < 3; i++){
        for(int j=0; j < 3; j++){
            g_Kattract(i,j) = tmpV4[i*3 + j];
        }
    }
    std::cout << "planner/attractive_gain: " << g_Kattract << std::endl;

    std::vector<double> tmpV5;
    nh.getParam("controller/task_stiffness", tmpV5);
    g_taskStiff = Eigen::MatrixXd::Zero(3, 3);
    for(int i=0; i < 3; i++){
        for(int j=0; j < 3; j++){
            g_taskStiff(i,j) = tmpV5[i*3 + j];
        }
    }
    std::cout << "controller/task_stiffness: " << g_taskStiff << std::endl;

    std::vector<double> tmpV6;
    nh.getParam("controller/task_damping", tmpV6);
    g_taskDamp = Eigen::MatrixXd::Zero(3, 3);
    for(int i=0; i < 3; i++){
        for(int j=0; j < 3; j++){
            g_taskDamp(i,j) = tmpV6[i*3 + j];
        }
    }
    std::cout << "controller/task_damping: " << g_taskDamp << std::endl;

    std::vector<double> tmpV7;
    nh.getParam("controller/joint_stiffness", tmpV7);
    g_jointStiff = Eigen::MatrixXd::Zero(g_numJoints, g_numJoints);
    for(int i=0; i < g_numJoints; i++){
        for(int j=0; j < g_numJoints; j++){
            g_jointStiff(i,j) = tmpV7[i*g_numJoints + j];
        }
    }
    std::cout << "controller/joint_stiffness: " << g_jointStiff << std::endl;

    std::vector<double> tmpV8;
    nh.getParam("controller/joint_damping", tmpV8);
    g_jointDamp = Eigen::MatrixXd::Zero(g_numJoints, g_numJoints);
    for(int i=0; i < g_numJoints; i++){
        for(int j=0; j < g_numJoints; j++){
            g_jointDamp(i,j) = tmpV8[i*g_numJoints + j];
        }
    }
    std::cout << "controller/joint_damping: " << g_jointDamp << std::endl;

    std::vector<double> tmpV9;
    nh.getParam("controller/nullspace_attractive", tmpV9);
    g_nullspaceAttract = Eigen::MatrixXd::Zero(g_numJoints, g_numJoints);
    for(int i=0; i < g_numJoints; i++){
        for(int j=0; j < g_numJoints; j++){
            g_nullspaceAttract(i,j) = tmpV9[i*g_numJoints + j];
        }
    }
    std::cout << "controller/nullspace_attractive: " << g_nullspaceAttract << std::endl;

    std::vector<double> tmpV10;
    nh.getParam("robot/torque_limit", tmpV10);
    g_torqueLimits = Eigen::VectorXd::Zero(tmpV10.size(), 1);
    for(int i=0; i < tmpV10.size(); i++){
        g_torqueLimits(i) = tmpV10[i];
    }
    std::cout << "robot/torque_limit: " << g_torqueLimits << std::endl;
        

    // Initialise ROS Server for target_pose
	ros::ServiceServer motion_planner_service = nh.advertiseService("/planner/set_hand_target", PlannerService); 

	// Init robot state subscriber
	robot_state_subscriber = nh.subscribe(joint_subscriber_topic, 1, robotStateCallBack);

	// Init robot pose publisher
	ros::Publisher torque_publisher = nh.advertise<std_msgs::Float64MultiArray>(torque_publisher_topic, 1);
	ros::Publisher finger_position_publisher = nh.advertise<std_msgs::Float64MultiArray>(finger_publisher_topic, 1);

	// Set Control loop rate
    int fCycle_Hz;
    nh.getParam("control_freq", fCycle_Hz);
    std::cout << "control_freq: " << fCycle_Hz << std::endl;
    double dtCycle_S = 1.0f/fCycle_Hz;
	ros::Rate loopRate(fCycle_Hz);

    // Get Robot Model
    Robot robot(urdf_file_name, g_numJoints);
    std::cout << "Start Config Global Vars" << std::endl;
    g_jointPos.resize(g_numJoints);
    g_jointVel.resize(g_numJoints);
    g_jointEffort.resize(g_numJoints);
    g_jointPos = g_initJointConfig;
    g_jointVel = Eigen::VectorXd::Zero(g_numJoints, 1);
    g_jointEffort = Eigen::VectorXd::Zero(g_numJoints, 1);

    //Initialize Planner
    PotentialFieldPlanner VPF(g_Kattract);
    g_VPF = VPF;
    std::cout << "Start Pose: " << robot.forwardKinematics(g_jointPos, g_jointVel) << std::endl;
    g_VPF.setGoalPosition(robot.forwardKinematics(g_jointPos, g_jointVel));

    //Initialize Inverse Dynamics Controller
    InverseDynamicsController ID_controller(g_taskStiff, g_taskDamp, g_jointStiff, g_jointDamp, dtCycle_S);
    ID_controller.useNullspace(g_useNullspace);
    ID_controller.setNullspaceGoal(g_nullspaceTargetConfig);
    ID_controller.setNullspacePlannerGain(g_nullspaceAttract);
    ID_controller.setTorqueLimits(g_torqueLimits);

    // Published data holder
	std_msgs::Float64MultiArray joint_effort_msg;
    joint_effort_msg.data.resize(g_numJoints);

    std_msgs::Float64MultiArray finger_position_msg;
    finger_position_msg.data.resize(g_fingerTargetConfig.size());
    for(int i = 0; i < g_fingerTargetConfig.size(); i++){
        finger_position_msg.data[i] = g_fingerTargetConfig[i];
    }

    // Publisher and Control cycle
	while(ros::ok()){
        std::cout << "start of loop" << std::endl;

        // Publish Finger Positions
        finger_position_publisher.publish(finger_position_msg);
        std::cout << "Published gripper: " << g_fingerTargetConfig << std::endl;

        // Potential Field planner to goal
        g_currXYZ = robot.forwardKinematics(g_jointPos, g_jointVel);
        std::cout << "FK: " << g_currXYZ << std::endl;

        Eigen::VectorXd currVelXYZ = robot.JacobianVel(g_jointPos, g_jointVel) * g_jointVel;
        std::cout << "J: " << currVelXYZ << std::endl;

        Eigen::VectorXd taskVelRef = g_VPF.nextAction(g_currXYZ);
        std::cout << "VPF: " << taskVelRef << std::endl;
        Eigen::VectorXd taskPosRef = g_currXYZ + taskVelRef * dtCycle_S;
        Eigen::VectorXd taskAccRef = (taskVelRef - currVelXYZ) / dtCycle_S;
        

        // Task Space Controller
        Eigen::VectorXd joint_torques = ID_controller.computeTorques(
            robot,
            taskPosRef,
            taskVelRef,
            taskAccRef,
            g_currXYZ,
            currVelXYZ,
            g_jointPos,
            g_jointVel
        );
        std::cout << "torques: " << joint_torques << std::endl;

        joint_effort_msg.data.resize(g_numEffortJoints);
        for(int i = 0; i < g_numEffortJoints; i++){
            joint_effort_msg.data[i] = joint_torques[i];
        }

        torque_publisher.publish(joint_effort_msg);

        ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

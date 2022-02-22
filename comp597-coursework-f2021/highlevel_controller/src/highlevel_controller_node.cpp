#include <highlevel_controller/robot.hpp>
#include <highlevel_controller/potential_field_planner.hpp>
#include <highlevel_controller/inverse_kinematics_controller.hpp>
// #include <highlevel_controller/inverse_dynamics_controller.hpp>

#include <highlevel_controller/target_pose.h>
#include <highlevel_controller/pick_and_place.h>

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
int g_numArmJoints;
int g_endEffectorJointID;
int g_numGripperJoints;
bool g_useNullspace;
float g_errorTol;
Eigen::VectorXd g_initJointConfig;
Eigen::VectorXd g_fingerTargetConfig;
Eigen::VectorXd g_nullspaceTargetConfig;
Eigen::VectorXd g_limits;
Eigen::VectorXd g_fingerOpenConfig;
Eigen::VectorXd g_fingerCloseConfig;
Eigen::VectorXd g_grabPose;
Eigen::VectorXd g_targetPose;
Eigen::VectorXd g_homePose;

// Controller Parameters
Eigen::MatrixXd g_Kattract;
Eigen::MatrixXd g_Kgripper;
double g_gripperMaxVel;

// Planner Parameters
PotentialFieldPlanner g_jointVPF;
PotentialFieldPlanner g_gripperVPF;
Eigen::VectorXd g_planJointPos;
Eigen::VectorXd g_planGripperPos;


// Robot State Feedback
Eigen::VectorXd g_jointPos;
Eigen::VectorXd g_jointVel;
Eigen::VectorXd g_jointEffort;
Eigen::VectorXd g_gripperPos;
Eigen::VectorXd g_gripperVel;
Eigen::VectorXd g_gripperEffort;

// Task Space Target and Current states
Eigen::VectorXd g_currXYZ = Eigen::VectorXd::Zero(6, 1);
Eigen::VectorXd g_desXYZ = Eigen::VectorXd::Zero(6, 1);

// State Machine Variables
enum GRASP_STATE {
    REACH,
    GRASP,
    MOVE,
    RELEASE,
    HOME
};

GRASP_STATE g_state = HOME;

/**
 * ROS Service for getting the providing the new goal for the manipulator controller
 */
bool PlannerService(
	highlevel_controller::target_pose::Request &req, 
	highlevel_controller::target_pose::Response &res
){
	ROS_INFO("New Target Pose Requested: x=%f, y=%f, z=%f, alpha=%f, beta=%f, gamma=%f", req.x, req.y, req.z, req.alpha, req.beta, req.gamma);
	// Save information to produce trajectory
	g_desXYZ(0) = req.x;
	g_desXYZ(1) = req.y;
	g_desXYZ(2) = req.z;
    g_desXYZ(3) = req.alpha;
    g_desXYZ(4) = req.beta;
    g_desXYZ(5) = req.gamma;

    g_jointVPF.setGoalPosition(g_desXYZ);

	res.success = true;

	return true;
}

/**
 * ROS Service for starting the Pick and Place operation
 */
bool PickPlaceService(
	highlevel_controller::pick_and_place::Request &req, 
	highlevel_controller::pick_and_place::Response &res
){
    ROS_INFO("Starting Pick and Place");
	g_state = REACH;

    res.success = true;

	return true;
}

/**
 * ROS Subscriber Callback to read robot state and save to global variables
 */ 
void robotStateCallBack(const sensor_msgs::JointState& msg){
	
    for(int i=0; i < g_numArmJoints; i++){
        g_jointPos(i) = msg.position[i];
        g_jointVel(i) = msg.velocity[i];
        g_jointEffort(i) = msg.effort[i];
    }

    for(int i= g_numArmJoints; i < (g_numGripperJoints + g_numArmJoints); i=i+2){
        int j = i - g_numArmJoints;
        g_gripperPos(j) = msg.position[i];
        g_gripperVel(j) = msg.velocity[i];
        g_gripperEffort(j) = msg.effort[i];
        g_gripperPos(j+1) = msg.position[i+3];
        g_gripperVel(j+1) = msg.velocity[i+3];
        g_gripperEffort(j+1) = msg.effort[i+3];

    }
}

Eigen::VectorXd loadRosParamVector(ros::NodeHandle nh, std::string paramName){
    std::vector<double> tmp;
    nh.getParam(paramName, tmp);
    Eigen::VectorXd vec = Eigen::VectorXd::Zero(tmp.size(), 1);
    for(int i=0; i < tmp.size(); i++){
        vec[i] = tmp[i];
    }
    std::cout << paramName << ": " << vec << std::endl;

    return vec;
}

Eigen::MatrixXd loadRosParamMatrix(ros::NodeHandle nh, std::string paramName){
    std::vector<double> tmp;
    nh.getParam(paramName, tmp);
    uint dim = sqrt(tmp.size());
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(dim, dim);
    for(int i=0; i < dim; i++){
        for(int j=0; j < dim; j++){
            mat(i,j) = tmp[i*dim + j];
        }
    }
    std::cout << paramName << ": " << mat << std::endl;

    return mat;
}

int main(int argc, char** argv) {

    // Init ROS Relationships
	ros::init(argc, argv, "highlevel_controller");
    ros::NodeHandle nh;

    // Load rosparameters
    std::string joint_publisher_topic;
    nh.getParam("joint_publisher_topic", joint_publisher_topic);

    std::string joint_subscriber_topic;
    nh.getParam("joint_subscriber_topic", joint_subscriber_topic);

    std::string finger_publisher_topic;
    nh.getParam("finger_publisher_topic", finger_publisher_topic);

    std::string urdf_file_name;
    nh.getParam("urdf_file", urdf_file_name);

    nh.getParam("use_nullspace", g_useNullspace);
    nh.getParam("num_joints", g_numJoints);
    nh.getParam("num_arm_joints", g_numArmJoints);
    nh.getParam("end_effector_joint_id", g_endEffectorJointID);
    nh.getParam("num_gripper_joints", g_numGripperJoints);
    
    g_initJointConfig = loadRosParamVector(nh, "robot/init_joint_config");
    g_fingerTargetConfig = loadRosParamVector(nh, "robot/finger_target_config");
    g_nullspaceTargetConfig = loadRosParamVector(nh, "robot/nullspace_target_config");
    g_Kattract = loadRosParamMatrix(nh, "planner/attractive_gain");
    g_limits = loadRosParamVector(nh, "robot/joint_velocity_limit");
    g_fingerOpenConfig = loadRosParamVector(nh, "robot/finger_open_config");
    g_fingerCloseConfig = loadRosParamVector(nh, "robot/finger_close_config");
    g_grabPose = loadRosParamVector(nh, "pick_and_place/grab_pose");
    g_targetPose = loadRosParamVector(nh, "pick_and_place/target_pose");
    g_homePose = loadRosParamVector(nh, "pick_and_place/home_pose");
    g_Kgripper = loadRosParamMatrix(nh, "planner/gripper_gain");

    nh.getParam("pick_and_place/gripper_max_vel", g_gripperMaxVel);
    nh.getParam("pick_and_place/pose_error_tolerance", g_errorTol);

    // Initialise ROS Server for target_pose
	ros::ServiceServer motion_planner_service = nh.advertiseService("/planner/set_hand_target", PlannerService); 
	ros::ServiceServer pick_place_service = nh.advertiseService("/highlevel_controller/pick_and_place", PickPlaceService); 

	// Init robot state subscriber
	robot_state_subscriber = nh.subscribe(joint_subscriber_topic, 1, robotStateCallBack);

	// Init robot pose publisher
	ros::Publisher joint_publisher = nh.advertise<std_msgs::Float64MultiArray>(joint_publisher_topic, 1);
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
    g_jointPos.resize(g_numArmJoints);
    g_jointVel.resize(g_numArmJoints);
    g_jointEffort.resize(g_numArmJoints);
    g_jointPos = g_initJointConfig;
    g_planJointPos = g_initJointConfig;
    g_jointVel = Eigen::VectorXd::Zero(g_numArmJoints, 1);
    g_jointEffort = Eigen::VectorXd::Zero(g_numArmJoints, 1);
    g_gripperPos.resize(g_numGripperJoints);
    g_gripperVel.resize(g_numGripperJoints);
    g_gripperEffort.resize(g_numGripperJoints);
    g_gripperPos = g_fingerOpenConfig;
    g_planGripperPos = g_fingerOpenConfig;
    g_gripperVel = Eigen::VectorXd::Zero(g_numGripperJoints, 1);
    g_gripperEffort = Eigen::VectorXd::Zero(g_numGripperJoints, 1);

    //Initialize Planners
    PotentialFieldPlanner VPF(g_Kattract);
    g_jointVPF = VPF;
    std::cout << "Start Pose: " << robot.forwardKinematics(g_jointPos, g_jointVel) << std::endl;
    g_jointVPF.setGoalPosition(robot.forwardKinematics(g_jointPos, g_jointVel));

    PotentialFieldPlanner VPF2(g_Kgripper);
    g_gripperVPF = VPF2;
    g_gripperVPF.setGoalPosition(g_fingerOpenConfig);

    //Initialize Inverse Kinematics Controller
    InverseKinematicsController IK_controller(dtCycle_S);
    IK_controller.useNullspace(g_useNullspace);
    IK_controller.setNullspaceGoal(g_nullspaceTargetConfig);
    IK_controller.setJointVelocityLimits(g_limits);

    // Published data holder
	std_msgs::Float64MultiArray joint_msg;
    joint_msg.data.resize(g_numArmJoints);

    std_msgs::Float64MultiArray finger_position_msg;
    finger_position_msg.data.resize(g_fingerTargetConfig.size());

    // Publisher and Control cycle
	while(ros::ok()){
        std::cout << "Start of loop | State " << g_state << std::endl;

        switch (g_state){
            case HOME:
                g_jointVPF.setGoalPosition(g_homePose);
                g_gripperVPF.setGoalPosition(g_fingerOpenConfig);
                break;

            case REACH:
                g_jointVPF.setGoalPosition(g_grabPose);
                g_gripperVPF.setGoalPosition(g_fingerOpenConfig);
                g_currXYZ = robot.forwardKinematics(g_planJointPos);
                if ( 
                    ((g_currXYZ - g_grabPose).norm() < g_errorTol) 
                    && 
                    ((g_planGripperPos - g_fingerOpenConfig).norm() < g_errorTol) 
                ) {
                    g_state = GRASP;
                }
                break;

            case GRASP:
                g_jointVPF.setGoalPosition(g_grabPose);
                g_gripperVPF.setGoalPosition(g_fingerCloseConfig);
                g_currXYZ = robot.forwardKinematics(g_planJointPos);
                if ( 
                    ((g_currXYZ - g_grabPose).norm() < g_errorTol)
                    &&
                    ((g_planGripperPos - g_fingerCloseConfig).norm() < g_errorTol)
                ){
                    g_state = MOVE;
                }
                break;

            case MOVE:
                g_jointVPF.setGoalPosition(g_targetPose);
                g_gripperVPF.setGoalPosition(g_fingerCloseConfig);
                g_currXYZ = robot.forwardKinematics(g_planJointPos);
                if (
                    ((g_currXYZ - g_targetPose).norm() < g_errorTol)
                    &&
                    ((g_planGripperPos - g_fingerCloseConfig).norm() < g_errorTol)
                ){
                    g_state = RELEASE;
                }
                break;
            
            case RELEASE:
                g_jointVPF.setGoalPosition(g_targetPose);
                g_gripperVPF.setGoalPosition(g_fingerOpenConfig);
                g_currXYZ = robot.forwardKinematics(g_planJointPos);
                if (
                    ((g_currXYZ - g_targetPose).norm() < g_errorTol)
                    &&
                    ((g_planGripperPos - g_fingerOpenConfig).norm() < g_errorTol)
                ){
                    g_state = HOME;
                }
                break;
        }

        // Potential Field planner to goal
        g_currXYZ = robot.forwardKinematics(g_planJointPos);
        std::cout << "FK: " << g_currXYZ << std::endl;

        Eigen::VectorXd taskVelRef = g_jointVPF.nextAction(g_currXYZ);
        std::cout << "Joint VPF: " << taskVelRef << std::endl;
        Eigen::VectorXd taskPosRef = g_currXYZ + taskVelRef * dtCycle_S;
        
        // Task Space Controller
        Eigen::VectorXd joint_velocities = IK_controller.computeJointVelocity(
            robot,
            taskPosRef,
            taskVelRef,
            g_jointPos,
            g_jointVel
        );

        // Check torque limits
        double max_ratio = 1.0f;
        for(int i = 0; i < g_limits.size(); i++){
            double ratio = abs(joint_velocities[i] / g_limits[i]);
            if(ratio > max_ratio){
                max_ratio = ratio;
            }
        }

        if(max_ratio > 1.0f){
            ROS_WARN("Joint Velocity Exceed Limits! Scaling down for highest to be at maximum.");
            for(int i = 0; i < g_limits.size(); i++){
                joint_velocities[i] = joint_velocities[i] * (1/max_ratio);
            }
        }
        std::cout << "Joint Vel: " << joint_velocities << std::endl;
        
        g_planJointPos = g_planJointPos + joint_velocities * dtCycle_S;
        std::cout << "Joint Pos: " << g_planJointPos << std::endl;

        joint_msg.data.resize(g_numArmJoints);
        for(int i = 0; i < g_numArmJoints; i++){
            joint_msg.data[i] = g_planJointPos[i];
        }

        // Publish Joint torques
        joint_publisher.publish(joint_msg);
    
    
        // Actuate Gripper
        Eigen::VectorXd gripperVelRef = g_gripperVPF.nextAction(g_planGripperPos);
        std::cout << "Gripper VPF: " << gripperVelRef << std::endl;
        
        // Check gripper vel limits
        max_ratio = 1.0f;
        for(int i = 0; i < gripperVelRef.size(); i++){
            double ratio = abs(gripperVelRef[i] / g_gripperMaxVel);
            if(ratio > max_ratio){
                max_ratio = ratio;
            }
        }

        if(max_ratio > 1.0f){
            for(int i = 0; i < gripperVelRef.size(); i++){
                gripperVelRef[i] = gripperVelRef[i] * (1/max_ratio);
            }
        }

        // Get new reference position
        g_planGripperPos = g_planGripperPos + gripperVelRef * dtCycle_S;

        for(int i = 0; i < g_planGripperPos.size(); i++){
            finger_position_msg.data[i] = g_planGripperPos[i];
        }
    
        // Publish Finger Positions
        finger_position_publisher.publish(finger_position_msg);
        std::cout << "Published gripper: " << g_planGripperPos << std::endl;


        ros::spinOnce();
		loopRate.sleep();
	}

	return 0;
}

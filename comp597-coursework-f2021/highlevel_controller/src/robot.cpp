#include <highlevel_controller/robot.hpp>

Eigen::MatrixXd Robot::pseudoInverse(Eigen::MatrixXd M){
    return M.completeOrthogonalDecomposition().pseudoInverse();
}

Robot::Robot(std::string urdf_file, int numJoints){
    pinocchio::urdf::buildModel(urdf_file, _model, false);				// read the URDF file
    pinocchio::Data data(_model);
    _data = data;
    _numJoints = numJoints;
}

Robot::~Robot(){}

Eigen::VectorXd Robot::forwardKinematics(Eigen::VectorXd q){
    pinocchio::forwardKinematics(_model, _data, q);
    pinocchio::SE3 currPose = _data.oMi[_numJoints];   // end-effector pose
    // return currPose.translation();
    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6,1);
    pose.head<3>() = currPose.translation();
    pose.tail<3>() = pinocchio::rpy::matrixToRpy( _data.oMi[_numJoints].rotation() );
    return pose;
}

Eigen::VectorXd Robot::forwardKinematics(Eigen::VectorXd q, Eigen::VectorXd dq){
    pinocchio::forwardKinematics(_model, _data, q, dq);
    pinocchio::SE3 currPose = _data.oMi[_numJoints];   // end-effector pose

    Eigen::VectorXd pose = Eigen::VectorXd::Zero(6,1);
    pose.head<3>() = currPose.translation();
    pose.tail<3>() = pinocchio::rpy::matrixToRpy( _data.oMi[_numJoints].rotation() );
    return pose;
}

pinocchio::SE3 Robot::forwardKinematicsPose(Eigen::VectorXd q, Eigen::VectorXd dq){
    pinocchio::forwardKinematics(_model, _data, q, dq);
    return _data.oMi[_numJoints];   // end-effector pose
}

Eigen::MatrixXd Robot::Jacobian(Eigen::VectorXd q, Eigen::VectorXd dq){
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,_numJoints) ;
    pinocchio::computeAllTerms(_model, _data, q, dq) ;
    pinocchio::getJointJacobian(_model, _data, _numJoints, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian) ;
    return jacobian;
}

Eigen::MatrixXd Robot::JacobianVel(Eigen::VectorXd q, Eigen::VectorXd dq){
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6,_numJoints) ;
    pinocchio::computeAllTerms(_model, _data, q, dq) ;
    pinocchio::getJointJacobian(_model, _data, _numJoints, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian) ;
    return jacobian.topRows(3);
}

Eigen::MatrixXd  Robot::JacobianDot(Eigen::VectorXd q, Eigen::VectorXd dq){
    Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6, _numJoints);
    pinocchio::computeJointJacobiansTimeVariation(_model, _data, q, dq) ;
    pinocchio::getJointJacobianTimeVariation(_model, _data, _numJoints, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;
    return jacobian_dot;
}

Eigen::MatrixXd  Robot::JacobianVelDot(Eigen::VectorXd q, Eigen::VectorXd dq){
    Eigen::MatrixXd jacobian_dot = Eigen::MatrixXd::Zero(6,_numJoints) ;
    pinocchio::computeJointJacobiansTimeVariation(_model, _data, q, dq) ;
    pinocchio::getJointJacobianTimeVariation(_model, _data, _numJoints, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian_dot) ;
    return jacobian_dot.topRows(3);
}

Eigen::MatrixXd  Robot::MassMatrix(){
    return _data.M;
}
        
Eigen::MatrixXd  Robot::NonLinearDynamics(){
    return _data.nle;
}

Eigen::MatrixXd Robot::NullSpace(Eigen::MatrixXd J){
    Eigen::MatrixXd invJ = pseudoInverse(J);
    return NullSpace(J, invJ);
}
Eigen::MatrixXd Robot::NullSpace(Eigen::MatrixXd J, Eigen::MatrixXd invJ){
    return Eigen::MatrixXd::Identity(_numJoints, _numJoints) - invJ * J;
}

Eigen::VectorXd Robot::inverseKinematicsVel(Eigen::VectorXd taskVel, Eigen::MatrixXd J){
    Eigen::MatrixXd pinvJ = pseudoInverse(J);
    return pinvJ * taskVel;
}

Eigen::VectorXd Robot::inverseKinematicsVel(Eigen::VectorXd taskVel, Eigen::MatrixXd J, Eigen::VectorXd qNull){
    Eigen::MatrixXd pinvJ = pseudoInverse(J);
    Eigen::VectorXd dq = pinvJ * taskVel;

    Eigen::MatrixXd N = NullSpace(J, pinvJ);
    return dq + N*qNull;
}      


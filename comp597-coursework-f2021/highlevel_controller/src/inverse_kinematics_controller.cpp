#include <highlevel_controller/inverse_kinematics_controller.hpp>


InverseKinematicsController::InverseKinematicsController(double dt){
    _dt = dt;
}

InverseKinematicsController::~InverseKinematicsController(){}

void InverseKinematicsController::useNullspace(bool active){
    _useNullspace = active;
}

void InverseKinematicsController::setNullspaceGoal(Eigen::VectorXd goal){
    _nullspaceTarget = goal;
}

void InverseKinematicsController::setJointVelocityLimits(Eigen::VectorXd limits){
    _limits = limits;
}

Eigen::VectorXd InverseKinematicsController::computeTaskJointVelocity(
    Robot r,
    Eigen::VectorXd x_ref,
    Eigen::VectorXd dx_ref,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb    
){   
    Eigen::MatrixXd J = r.Jacobian(q_fb, dq_fb);
    Eigen::MatrixXd pinvJ = r.pseudoInverse(J);

    return pinvJ * dx_ref;
}

Eigen::VectorXd InverseKinematicsController::computeNullspaceJointVelocity(
    Robot r,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb    
){
    Eigen::MatrixXd J = r.Jacobian(q_fb, dq_fb);
    Eigen::MatrixXd pinvJ = r.pseudoInverse(J);

    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(q_fb.size(), q_fb.size()) - pinvJ * J;

    return N * (_nullspaceTarget - q_fb);
}

Eigen::VectorXd InverseKinematicsController::computeJointVelocity(
    Robot r,
    Eigen::VectorXd x_ref,
    Eigen::VectorXd dx_ref,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb  
){

    Eigen::VectorXd dq = computeTaskJointVelocity(
            r,
            x_ref,
            dx_ref,
            q_fb,
            dq_fb  
        );
    
    std::cout << "task joint velocity: " << dq << std::endl;

    if(_useNullspace){
        Eigen::VectorXd dq_null = computeNullspaceJointVelocity(
            r,
            q_fb,
            dq_fb
        );

        std::cout << "null joint velocity: " << dq_null << std::endl;

        dq = dq + dq_null;
    }
    
    return dq;
        
}
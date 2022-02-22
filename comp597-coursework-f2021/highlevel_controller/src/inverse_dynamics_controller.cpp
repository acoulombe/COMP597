#include <highlevel_controller/inverse_dynamics_controller.hpp>


InverseDynamicsController::InverseDynamicsController(Eigen::MatrixXd Kt, Eigen::MatrixXd Dt, Eigen::MatrixXd Kj, Eigen::MatrixXd Dj, double dt){
    _K_task = Kt;
    _D_task = Dt;
    _K_joint = Kj;
    _D_joint = Dj;
    _dt = dt;
}

InverseDynamicsController::~InverseDynamicsController(){}

void InverseDynamicsController::useNullspace(bool active){
    _useNullspace = active;
}

void InverseDynamicsController::setNullspaceGoal(Eigen::VectorXd goal){
    _VPF.setGoalPosition(goal);
}

void InverseDynamicsController::setNullspacePlannerGain(Eigen::MatrixXd K){
    _VPF.setKattractGain(K);
}

void InverseDynamicsController::setTorqueLimits(Eigen::VectorXd limits){
    _limits = limits;
}


Eigen::VectorXd InverseDynamicsController::computeTaskTorques(
    Robot r,
    Eigen::VectorXd x_ref,
    Eigen::VectorXd dx_ref,
    Eigen::VectorXd ddx_ref,
    Eigen::VectorXd x_fb,
    Eigen::VectorXd dx_fb,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb    
){

    // Eigen::VectorXd ddx_cmd = ddx_ref + _D_task*(dx_ref - dx_fb) + _K_task*(x_ref - x_fb);
    Eigen::VectorXd ddx_cmd = _D_task*(dx_ref - dx_fb) + _K_task*(x_ref - x_fb);
    
    Eigen::MatrixXd J = r.Jacobian(q_fb, dq_fb);
    Eigen::MatrixXd dJ = r.JacobianDot(q_fb, dq_fb);
    Eigen::MatrixXd M = r.MassMatrix();
    Eigen::VectorXd h = r.NonLinearDynamics();
    Eigen::MatrixXd pinvJ_T = r.pseudoInverse(J.transpose());
    Eigen::MatrixXd pinvJ = r.pseudoInverse(J);

    Eigen::MatrixXd lambda = pinvJ_T * M * pinvJ;
    Eigen::VectorXd eta = pinvJ_T*h - lambda * dJ * dq_fb;
    Eigen::VectorXd force_cmd = lambda * ddx_cmd + eta;

    return J.transpose() * force_cmd;

}

Eigen::VectorXd InverseDynamicsController::computeNullspaceTorques(
    Robot r,
    Eigen::VectorXd q_ref,
    Eigen::VectorXd dq_ref,
    Eigen::VectorXd ddq_ref,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb    
){

    // Eigen::VectorXd ddq_cmd = ddq_ref + _D_joint*(dq_ref - dq_fb) + _K_joint*(q_ref - q_fb);
    Eigen::VectorXd ddq_cmd = _D_joint*(dq_ref - dq_fb) + _K_joint*(q_ref - q_fb);
    
    Eigen::MatrixXd J = r.Jacobian(q_fb, dq_fb);
    Eigen::MatrixXd M = r.MassMatrix();
    Eigen::VectorXd h = r.NonLinearDynamics();

    Eigen::VectorXd t_joint = M * ddq_cmd + h;

    Eigen::MatrixXd JT = J.transpose();
    Eigen::MatrixXd invM = M.inverse();

    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(q_ref.size(), q_ref.size()) - JT * (J * invM * JT).inverse() * J * invM;

    return P * t_joint;
}

Eigen::VectorXd InverseDynamicsController::computeTorques(
    Robot r,
    Eigen::VectorXd x_ref,
    Eigen::VectorXd dx_ref,
    Eigen::VectorXd ddx_ref,
    Eigen::VectorXd x_fb,
    Eigen::VectorXd dx_fb,
    Eigen::VectorXd q_fb,
    Eigen::VectorXd dq_fb  
){

    Eigen::VectorXd t = computeTaskTorques(
            r,
            x_ref,
            dx_ref,
            ddx_ref,
            x_fb,
            dx_fb,
            q_fb,
            dq_fb  
        );
    
    std::cout << "task torques: " << t << std::endl;

    if(_useNullspace){
        Eigen::VectorXd dq_ref = _VPF.nextAction(q_fb);
        Eigen::VectorXd q_ref = q_fb + dq_ref * _dt;
        Eigen::VectorXd ddq_ref = (dq_ref - dq_fb) / _dt;
        Eigen::VectorXd t_null = computeNullspaceTorques(
            r,
            q_ref,
            dq_ref,
            ddq_ref,
            q_fb,
            dq_fb
        );

        std::cout << "null torques: " << t_null << std::endl;

        t = t + t_null;
    }
    
    return t;
        
}
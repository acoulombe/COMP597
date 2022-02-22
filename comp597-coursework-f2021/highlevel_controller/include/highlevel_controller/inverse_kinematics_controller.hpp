#ifndef _INVERSE_KINEMATICS_CONTROLLER_HPP
#define _INVERSE_KINEMATICS_CONTROLLER_HPP

#include <highlevel_controller/robot.hpp>

class InverseKinematicsController{

    private:
        double _dt;
        bool _useNullspace;
        Eigen::VectorXd _nullspaceTarget;
        Eigen::VectorXd _limits;
        
        Eigen::VectorXd computeTaskJointVelocity(
            Robot r,
            Eigen::VectorXd x_ref,
            Eigen::VectorXd dx_ref,  
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb  
        );

        Eigen::VectorXd computeNullspaceJointVelocity(
            Robot r,
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb    
        );

    public:
        InverseKinematicsController(double dt);
        virtual ~InverseKinematicsController();

        void useNullspace(bool active);
        void setNullspaceGoal(Eigen::VectorXd goal);
        void setJointVelocityLimits(Eigen::VectorXd limits);
        
        Eigen::VectorXd computeJointVelocity(
            Robot r,
            Eigen::VectorXd x_ref,
            Eigen::VectorXd dx_ref,
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb  
        );

};

#endif // _INVERSE_KINEMATICS_CONTROLLER_HPP
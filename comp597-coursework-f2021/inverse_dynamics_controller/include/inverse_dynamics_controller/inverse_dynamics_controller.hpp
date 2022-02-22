#ifndef _INVERSE_DYNAMICS_CONTROLLER_HPP
#define _INVERSE_DYNAMICS_CONTROLLER_HPP

#include <inverse_dynamics_controller/robot.hpp>
#include <inverse_dynamics_controller/potential_field_planner.hpp>

class InverseDynamicsController{

    private:
        double _dt;
        bool _useNullspace;
        Eigen::MatrixXd _K_task;
        Eigen::MatrixXd _D_task;
        Eigen::MatrixXd _K_joint;
        Eigen::MatrixXd _D_joint;

        Eigen::VectorXd _limits;

        PotentialFieldPlanner _VPF;
        
        Eigen::VectorXd computeTaskTorques(
            Robot r,
            Eigen::VectorXd x_ref,
            Eigen::VectorXd dx_ref,
            Eigen::VectorXd ddx_ref,
            Eigen::VectorXd x_fb,
            Eigen::VectorXd dx_fb,
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb  
        );

        Eigen::VectorXd computeNullspaceTorques(
            Robot r,
            Eigen::VectorXd q_ref,
            Eigen::VectorXd dq_ref,
            Eigen::VectorXd ddq_ref,
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb    
        );

    public:
        InverseDynamicsController(Eigen::MatrixXd Kt, Eigen::MatrixXd Dt, Eigen::MatrixXd Kj, Eigen::MatrixXd Dj, double dt);
        virtual ~InverseDynamicsController();

        void useNullspace(bool active);
        void setNullspacePlannerGain(Eigen::MatrixXd K);
        void setNullspaceGoal(Eigen::VectorXd goal);

        void setTorqueLimits(Eigen::VectorXd limits);
        
        Eigen::VectorXd computeTorques(
            Robot r,
            Eigen::VectorXd x_ref,
            Eigen::VectorXd dx_ref,
            Eigen::VectorXd ddx_ref,
            Eigen::VectorXd x_fb,
            Eigen::VectorXd dx_fb,
            Eigen::VectorXd q_fb,
            Eigen::VectorXd dq_fb  
        );

};

#endif // _INVERSE_DYNAMICS_CONTROLLER_HPP
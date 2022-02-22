#ifndef _POTENTIAL_FIELD_PLANNER_HPP
#define _POTENTIAL_FIELD_PLANNER_HPP

#include <Eigen/Dense>

class PotentialFieldPlanner{

    private:
        Eigen::MatrixXd _Kattract;
        Eigen::MatrixXd _Krepulse;
        Eigen::VectorXd _goal_position;

        Eigen::VectorXd AttractiveAction(Eigen::VectorXd current_position);
        Eigen::VectorXd RepulsiveAction(Eigen::VectorXd current_position);

    public:
        PotentialFieldPlanner();
        PotentialFieldPlanner(Eigen::MatrixXd Kattract);
        PotentialFieldPlanner(Eigen::MatrixXd Kattract, Eigen::MatrixXd Krepulse);
        virtual ~PotentialFieldPlanner();

        void setKattractGain(Eigen::MatrixXd K);
        void setKrepulseGain(Eigen::MatrixXd K);
        void setGoalPosition(Eigen::VectorXd goal);
        Eigen::VectorXd nextAction(Eigen::VectorXd current_position);
};

#endif //_POTENTIAL_FIELD_PLANNER_HPP
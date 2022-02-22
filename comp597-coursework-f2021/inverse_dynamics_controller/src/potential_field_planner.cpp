#include <inverse_dynamics_controller/potential_field_planner.hpp>
#include <iostream>

PotentialFieldPlanner::PotentialFieldPlanner(){}

PotentialFieldPlanner::PotentialFieldPlanner(Eigen::MatrixXd Kattract){
    _Kattract = Kattract;
}

PotentialFieldPlanner::PotentialFieldPlanner(Eigen::MatrixXd Kattract, Eigen::MatrixXd Krepulse){
    _Kattract = Kattract;
    _Krepulse = Krepulse;
}

PotentialFieldPlanner::~PotentialFieldPlanner(){}

void PotentialFieldPlanner::setKattractGain(Eigen::MatrixXd K){
    _Kattract = K;
}

void PotentialFieldPlanner::setKrepulseGain(Eigen::MatrixXd K){
    _Krepulse = K;
}

void PotentialFieldPlanner::setGoalPosition(Eigen::VectorXd goal){
    _goal_position = goal;
}

Eigen::VectorXd PotentialFieldPlanner::AttractiveAction(Eigen::VectorXd current_position){
    return _Kattract * (_goal_position - current_position);
}


Eigen::VectorXd PotentialFieldPlanner::RepulsiveAction(Eigen::VectorXd current_position){
    return Eigen::VectorXd::Zero(current_position.size(), 1);
}

Eigen::VectorXd PotentialFieldPlanner::nextAction(Eigen::VectorXd current_position){
    return AttractiveAction(current_position) + RepulsiveAction(current_position);
}

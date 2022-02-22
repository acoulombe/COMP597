#ifndef _ROBOT_HPP
#define _ROBOT_HPP

// Pinocchio Includes
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/math/rpy.hpp>

class Robot {

    private:
        pinocchio::Model _model;
        pinocchio::Data _data;
        int _numJoints;
    public:
        Robot(std::string urdf_file, int numJoints);
        virtual ~Robot();

        Eigen::VectorXd forwardKinematics(Eigen::VectorXd q);
        Eigen::VectorXd forwardKinematics(Eigen::VectorXd q, Eigen::VectorXd dq);
        pinocchio::SE3  forwardKinematicsPose(Eigen::VectorXd q, Eigen::VectorXd dq);

        Eigen::MatrixXd Jacobian(Eigen::VectorXd q, Eigen::VectorXd dq);
        Eigen::MatrixXd JacobianVel(Eigen::VectorXd q, Eigen::VectorXd dq);
        Eigen::MatrixXd JacobianDot(Eigen::VectorXd q, Eigen::VectorXd dq);
        Eigen::MatrixXd JacobianVelDot(Eigen::VectorXd q, Eigen::VectorXd dq);
        Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd M);

        Eigen::MatrixXd MassMatrix();
        Eigen::MatrixXd NonLinearDynamics();

        Eigen::MatrixXd NullSpace(Eigen::MatrixXd J);
        Eigen::MatrixXd NullSpace(Eigen::MatrixXd J, Eigen::MatrixXd invJ);

        Eigen::VectorXd inverseKinematicsVel(Eigen::VectorXd taskVel, Eigen::MatrixXd J);
        Eigen::VectorXd inverseKinematicsVel(Eigen::VectorXd taskVel, Eigen::MatrixXd J, Eigen::VectorXd qNull);        
};

#endif // _ROBOT_HPP
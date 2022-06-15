#pragma once

#include <vector>
#include <Eigen/Dense>

#include <robot_model/Model.hpp>

std::vector<double> jointStateToPosVec(state_representation::JointState jointState);
std::vector<double> eigenVecToStdVec(Eigen::VectorXd eigenVec);
Eigen::VectorXd stdVecToEigenVec(std::vector<double> stdVec);
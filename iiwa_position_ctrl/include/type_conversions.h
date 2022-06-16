/**
 * @file type_conversions.h
 * @author Loïc Niederhauser (loic.niederhauser@epfl.ch)
 * @brief Contains functions to perform conversions between different vector types.
 * @copyright Copyright (c) EPFL-LASA 2022
 */
#pragma once

#include <vector>
#include <Eigen/Dense>

#include <state_representation/space/joint/JointState.hpp>

/**
 * @brief Takes a joint state object and returns the position as vector
 * 
 * @param jointState The joint state to extract the joint position from.
 * @return std::vector<double> A standard vector containing the joint positions.
 */
std::vector<double> jointStateToPosVec(state_representation::JointState jointState);

/**
 * @brief Converts an eigen VectorXd to a standard c++ vector
 * 
 * @param eigenVec The eigen VectorXd to convert.
 * @return std::vector<double> The equivalent std::vector to the eigen VectorXd.
 */
std::vector<double> eigenVecToStdVec(Eigen::VectorXd eigenVec);

/**
 * @brief Converts a standard c++ vector ot eigen VectorXd
 * 
 * @param stdVec The standard vector to convert
 * @return Eigen::VectorXd The equivalent eigen VectorXd to the standard vector.
 */
Eigen::VectorXd stdVecToEigenVec(std::vector<double> stdVec);
/**
 * @file type_conversions.cpp
 * @author Loïc Niederhauser (loic.niederhauser@epfl.ch)
 * @brief Contains functions to perform conversions between different vector types.
 * @copyright Copyright (c) EPFL-LASA 2022
 */

#include "type_conversions.h"

std::vector<double> jointStateToPosVec(state_representation::JointState jointState){
    
    uint32_t nbElem = jointState.get_size();
    Eigen::VectorXd eigenVectPos = jointState.get_positions();
    std::vector<double> stdVectPos(nbElem);

    for(uint32_t i = 0; i < nbElem; ++i){
        stdVectPos[i] = eigenVectPos[i];
    }

    return stdVectPos;
}

std::vector<double> eigenVecToStdVec(Eigen::VectorXd eigenVec){
    uint32_t nbElem = eigenVec.size();
    std::vector<double> stdVec(nbElem);

    for(uint32_t i = 0; i < nbElem; ++i){
        stdVec[i] = eigenVec[i];
    }

    return stdVec;
}

Eigen::VectorXd stdVecToEigenVec(std::vector<double> stdVec){
    uint32_t nbElem = stdVec.size();
    Eigen::VectorXd eigenVec(nbElem);

    for(uint32_t i = 0; i < nbElem; ++i){
        eigenVec[i] = stdVec[i];
    }

    return eigenVec;
}
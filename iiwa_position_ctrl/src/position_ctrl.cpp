
/**
 * @file position_ctrl.cpp
 * @author Loïc Niederhauser (loic.niederhauser@epfl.ch)
 * @brief A file containing a ROS node capable of running an action server
 *        to go to specific positions in joint space
 * @copyright Copyright (c) EPFL-LASA 2022
 */
#include <chrono>
#include <thread>
#include <algorithm>

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>

#include <iiwa_position_msgs/goToJointPosGoal.h>
#include <iiwa_position_msgs/goToJointPosResult.h>
#include <iiwa_position_msgs/goToJointPosFeedback.h>

#include "position_ctrl.h"
#include "type_conversions.h"

PositionController::PositionController(const std::shared_ptr<ros::NodeHandle> &nh,
                                       const std::string robotName,
                                       const std::string actionServerName,
                                       const float actionFBFrq = 200.) : 
    nodeHandle_(nh),
    firstJointStateReceived_(false),
    goToJointPosAS_(*nh, actionServerName, boost::bind(&PositionController::goToJointPosCallback, this, _1), false),
    actionFBFrq_(actionFBFrq) {

    // Create a robot model
    std::string robotDescription;
    ROS_INFO("Readin robot param");
    if(!nodeHandle_->getParam("/robot_description", robotDescription)) {
        ROS_ERROR("Could not load parameter 'robot_description' from parameter server.");
        ros::shutdown();
    }
    std::string urdfPath = "/tmp/" + robotName + ".urdf";
    robot_model::Model::create_urdf_from_string(robotDescription, urdfPath);
    robotModel_ = std::make_unique<robot_model::Model>(robotName, urdfPath);
    jointState_ = state_representation::JointState(robotModel_->get_robot_name(), robotModel_->get_joint_frames());

    // Setup subs
    jointStateSub_ = nodeHandle_->subscribe<sensor_msgs::JointState>(
        "/" + robotName + "/joint_states", 1, &PositionController::jointStateCallback, this);

    // Setup pubs
    commandPub_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("/" + robotName + "/PositionController/command", 1);

    ROS_INFO("Starting action server");
    goToJointPosAS_.start();
}

void PositionController::setActionFBFrq(float newActionFBFrq){
    if(newActionFBFrq <= 0){
        ROS_ERROR("Cannot set action feedback frequency."
                  "Value must be strictly greater than 0");
    } else {
        actionFBFrq_ = newActionFBFrq;
    }
}

void PositionController::sendPosCommand(std::vector<double> posCmd) {
    std_msgs::Float64MultiArray positionCmdMsg;

    positionCmdMsg.data.resize(posCmd.size());
    for(uint32_t i = 0; i < posCmd.size(); ++i){
        positionCmdMsg.data[i] = ratio * posCmd[i];
    }

    commandPub_.publish(positionCmdMsg);
}

void PositionController::sendTrajPosCommand(std::vector<double> goalPos, std::vector<double> startPos, double progress){
    
    uint32_t nbJoints = goalPos.size();
    std::vector<double> posCmd(nbJoints);

    // Linear interpolation in joint space
    for(uint32_t i = 0; i < nbJoints; ++i){
        posCmd[i] = startPos[i] + (goalPos[i] - startPos[i]) * progress; 
    }

    // Send command
    this->sendPosCommand(posCmd);

}

void PositionController::stopRobot(){
    std::lock_guard<std::mutex> jointStateLock(jointStateMtx_);
    this->sendPosCommand(eigenVecToStdVec(jointState_.get_positions()));
}

void PositionController::jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg) {

    if(!firstJointStateReceived_){
        firstJointStateReceived_ = true;
    }

    // Update internal memory of the latest state
    std::lock_guard<std::mutex> jointSateLock(jointStateMtx_);
    jointState_.set_positions(msg->position);
    jointState_.set_velocities(msg->velocity);
    jointState_.set_torques(msg->effort);
}

void PositionController::goToJointPosCallback(const iiwa_position_msgs::goToJointPosGoalConstPtr &goal) {

    // Execution flags
    bool goalReached = false;
    bool goalValid = true;

    // Ros helpers
    ros::Rate FBRate(actionFBFrq_);
    Eigen::VectorXd currentError;
    ros::Time startTime;
    ros::Duration timeSinceStart;

    // Messages
    iiwa_position_msgs::goToJointPosFeedback feedbackMsg;
    iiwa_position_msgs::goToJointPosResult resultMsg;

    // Related to trajectory tracking
    double progress = 0;
    std::vector<double> startPose;

    // Get a lock for joint state. We will need it later
    std::unique_lock<std::mutex> jointStateLock(jointStateMtx_);
    jointStateLock.unlock();

    // Wait for first joint position before moving
    while (!firstJointStateReceived_){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Check goal valid
    goalValid = this->checkGoalValid(goal);

    if (goalValid){

        // Record mvt start time and pose
        startTime = ros::Time::now();
        jointStateLock.lock();
        startPose = eigenVecToStdVec(jointState_.get_positions());
        jointStateLock.unlock();

        // Feedback loop
        while (ros::ok()){
            
            // Check for preemption
            if(goToJointPosAS_.isPreemptRequested()){
                this->stopRobot();
                ROS_INFO("Action preempted");
                break;
            }

            // Check if goal is reached
            jointStateLock.lock();
            currentError = jointState_.get_positions() - stdVecToEigenVec(goal->goalPose);
            jointStateLock.unlock();
            goalReached = this->goalReached(currentError, goal->tol);

            // Break out when goal is reached
            if(goalReached){
                this->stopRobot();
                ROS_INFO("Goal reached"); 
                break;
            }

            // Format and send feedback
            feedbackMsg.nbJoints = robotModel_->get_number_of_joints();
            feedbackMsg.currentError = eigenVecToStdVec(currentError);
            goToJointPosAS_.publishFeedback(feedbackMsg);
            
            // Follow linear trajectory if goal time is specified
            if(goal->timeToGoal > 0){
                timeSinceStart = ros::Time::now() - startTime;
                progress = std::min(timeSinceStart.toSec() / goal->timeToGoal, 1.);
                this->sendTrajPosCommand(goal->goalPose, startPose, progress);
            
            // Else just go to goal as fast as possible
            } else {
                this->sendPosCommand(goal->goalPose);
            }

            // Sleep until next loop
            FBRate.sleep();
        }
    }

    // Result message
    resultMsg.nbJoints = robotModel_->get_number_of_joints();
    resultMsg.positionReached = goalReached;
    resultMsg.finalError = eigenVecToStdVec(currentError);
    resultMsg.goalValid = goalValid;

    if(goToJointPosAS_.isPreemptRequested()){
        goToJointPosAS_.setPreempted(resultMsg);
    } else {
        goToJointPosAS_.setSucceeded(resultMsg);
    }
}

bool PositionController::checkGoalValid(const iiwa_position_msgs::goToJointPosGoalConstPtr &goal){
    
    bool goalValid = true;

    // Check number of joints
    if (goal->nbJoints != robotModel_->get_number_of_joints()){
        ROS_ERROR("Goal has the wrong number of joints, aborting action.");
        goalValid = false;
    
    // Check if goal is in range
    } else {

        // Wrap goal in JointState for easy check
        state_representation::JointPositions goalJointPos = state_representation::JointState::Zero("", goal->nbJoints);
        goalJointPos.set_positions(goal->goalPose);

        // Check range
        if(!robotModel_->in_range(goalJointPos)){
            ROS_ERROR("Goal position outside of robot bounds, aborting action.");
            goalValid = false;
        }
    }

    return goalValid;
}

bool PositionController::goalReached(Eigen::VectorXd posDiff, std::vector<double> tol){
    bool goalIsReached = true;

    for (uint32_t i = 0; i < posDiff.size(); ++i)
    {
        if (abs(posDiff[i]) > abs(tol[i])){
            goalIsReached = false;
            break;
        }
    }
    return goalIsReached;
}




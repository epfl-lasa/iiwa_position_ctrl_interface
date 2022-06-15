/**
 * @file position_ctrl.h
 * @author Loïc Niederhauser (loic.niederhauser@epfl.ch)
 * @brief A file containing a ROS node capable of running an action server
 *        to go to specific positions in joint space
 * @copyright Copyright (c) EPFL-LASA 2022
 */
#pragma once

#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <robot_model/Model.hpp>
#include <actionlib/server/simple_action_server.h>

#include <iiwa_position_msgs/goToJointPosAction.h>

/**
 * @brief Ros Node to control the position of a robot through a ROS action.
 */
class PositionController {

public:
    /**
     * @brief Construct a new Position Controller object
     * 
     * @param nh shared pointer to node handle to use
     * @param robotName Name of the robot to control
     * @param actionServerName Name to give to the action server
     * @param actionFBFrq Frequency of the feedback in the action server [Hz]
     */
    PositionController(const std::shared_ptr<ros::NodeHandle> &nh,
                       const std::string robotName,
                       const std::string actionServerName,
                       const float actionFBFrq);

    /**
     * @brief Set the Action feedback frequency
     * 
     * @param newActionFBFrq The new feedback frequency to use for the action server
     */
    void setActionFBFrq(float newActionFBFrq);

protected:

    /**
     * @brief Send a position command in joint space to the robot
     * @details No checks are performed. It is up to the caller to make sure it's valid.
     * 
     * @param posCmd Postion command containing joints values to go to.
     */
    void sendPosCommand(std::vector<double> posCmd);

    /**
     * @brief Stops the robot
     * @details Does so by sending a command to go to the current position of the robot.
     *          Robot might keep moving a bit due to controller
     * 
     */
    void stopRobot();

    /**
     * @brief Callback safely updating the internal knowledge of 
     *        the current joint state of the class.
     * 
     * @param msg Message received from the robot containing the joint states.
     */
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);

    /**
     * @brief Callback run by the action server to go to a specific position
     * @details The callback takes into account preemption and will finish early if 
     *          the goal is not feasible.
     * 
     * @param goal Message containing the goal position
     */
    void goToJointPosCallback(const iiwa_position_msgs::goToJointPosGoalConstPtr &goal);

    /**
     * @brief Check if a goal message is valid regarding the robot model.
     * 
     * @param goal Goal message to be checked for validity
     * @return bool Whether or not the goal message is feasible and valid
     */
    bool checkGoalValid(const iiwa_position_msgs::goToJointPosGoalConstPtr &goal);

    /**
     * @brief Check if a goal position is reached in joint state.
     * @details Does so by checking if jointDifference between current and goal position
     *          is smaller that a tolerance value for each joints
     * 
     * @param jointDiff A vector containing the difference between current and goal position.
     *                  in joint space for each joint
     * @param tol A tolerance vector the same size as jointDiff, containing the tolerated error for each
     *        joint.
     * @return bool Whether or not the goal has been reached. 
     */
    bool goalReached(Eigen::VectorXd jointDiff, std::vector<double> tol);
    
    // Pointer to model and ros objects
    std::shared_ptr<ros::NodeHandle> nodeHandle_;
    std::shared_ptr<robot_model::Model> robotModel_;
    
    // Variables concerning the current joint state of the robot.
    std::mutex jointStateMtx_;
    state_representation::JointState jointState_;
    std::atomic_bool firstJointStateReceived_;

    // Subscriber and publishers
    ros::Subscriber jointStateSub_;
    ros::Publisher commandPub_;

    // Action server variables
    actionlib::SimpleActionServer<iiwa_position_msgs::goToJointPosAction> goToJointPosAS_; // Action server to go to joint position
    std::atomic<float> actionFBFrq_;
};
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

class PositionController {

public:
    PositionController(const std::shared_ptr<ros::NodeHandle> &nh,
                       const std::string robotName,
                       const std::string actionServerName,
                       const float actionFBFrq);

    void setActionFBFrq(float newActionFBFrq);

protected:

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
    void goToJointPosCallback(const iiwa_position_msgs::goToJointPosGoalConstPtr &goal);
    bool goalReached(Eigen::VectorXd jointDiff, std::vector<double> tol);

    std::shared_ptr<robot_model::Model> robotModel_;
    std::shared_ptr<ros::NodeHandle> nodeHandle_;

    std::mutex jointStateMtx_;
    state_representation::JointState jointState_;
    std::atomic_bool firstJointStateReceived_;

    ros::Subscriber jointStateSub_;
    ros::Publisher commandPub_;

    std::atomic<float> actionFBFrq_;
    actionlib::SimpleActionServer<iiwa_position_msgs::goToJointPosAction> goToJointPosAS_; // Action server to go to joint position
};
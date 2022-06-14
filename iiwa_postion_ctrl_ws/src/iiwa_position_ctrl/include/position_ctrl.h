#pragma once

#include <mutex>
#include <atomic>
#include <Eigen/Dense>

#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <robot_model/Model.hpp>
#include <actionlib/server/simple_action_server.h>

#include <iiwa_position_msgs/goToJointPos.h>

class PositionController {

public:

    PositionController(const std::shared_ptr<ros::NodeHandle> &nh, 
                       const std::string robotName, 
                       const std::string actionServerName);

    setActionFBFrq(float newActionFBFrq);

protected:

        void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
        void goToJointPosCallback(const iiwa_position_msgs::goToJointPos& goal);
        bool goalReached(state_representation::JointPositions jointDiff, std_msgs::Float64MultiArray tol);

        std::shared_ptr<robot_model::Model> robotModel_;
        std::shared_ptr<ros::NodeHandle> nodeHandle_;

        std::mutex jointStateMtx_;
        state_representation::JointState jointState_;
        std::atomic_bool firstJointStateRecived_;

        ros::Subscriber jointStateSub_;
        ros::Publisher commandPub_;

        std::atomic<float> actionFBFrq_;
        actionlib::SimpleActionServer<iiwa_position_msgs::goToJointPos> goToJointPosAS_; // Action server to go to joint position
};

#include <chrono>
#include <thread>

#include <std_msgs/Float64MultiArray.h>

#include "position_ctrl.h"

PositionController::PositionController(const std::shared_ptr<ros::NodeHandle> &nh,
                                       const std::string robotName,
                                       const std::string actionServerName,
                                       const float actionFBFrq = 200.) : 
    nodeHandle_(nh),
    goToJointPosAS_(nh, actionServerName, boost::bind(&PositionController::goToJointPosCallback, this, _1), false),
    firstJointStateReceived_(false),
    serviceFBFrq_(actionFBFrq) {

    // Create a robot model
    std::string robotDescription;
    if(!nodeHandle_->getParam("/robot_description", robotDescription)) {
        ROS_ERROR("Could not loat parameter 'robot_description' from parameter server.");
        ros::shutdown();
    }
    std::string urdfPath = "/tmp/" + robotName + ".urdf";
    robot_model::Model::create_urdf_from_string(robotDescription, urdfPath);
    robotModel_ = std::make_unique<robot_model::Model>(robotName, urdfPath);
    jointState_ = jointState(robotModel_->get_robot_name(), robotModel_->get_joint_frames());

    // Setup subs
    jointStateSub_ = nodeHandle_->subscribe<std_msgs::JointState>(
        "/" + robotName + "/joint_states", 1, &PositionController::jointStateCallback, this);

    // Setup pubs
    commandPub_ = nodeHandle_->advertise<std_msgs::Float64MultiArray>("/" + robotName + "/PositionController/command", 1);

    // Wait for first joint position before starting the server
    while (!firstJointStateReceived_){
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    goToJointPosAS_.start();
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
    ros::Rate FBRate(1. / serviceFBFrq_);
    state_representation::JointState currentError;

    // Messages
    std_msgs::Float64MultiArray positionCmdMsg;
    iiwa_position_msgs::goToJointPosFeedback feedbackMsg;
    iiwa_position_msgs::goToJointPosResult resultMsg;

    // Check number of joints
    if (goal.nbJoints != robotModel_->get_number_of_joints()){
        ROS_ERROR("Goal has the wrong number of joints, aborting action.");
        goalValid = false;
    }

    // Check if the goal is in range
    state_representation::JointPositions goalJointPos = state_representation::JointState::Zero("", goal.nbJoints);
    goalJointPos.set_positions(goal->goalPose);
    if(!robotModel_->in_range(goalJointPos)){
        ROS_ERROR("Goal position outside of robot bounds, aborting action.");
        goalValid = false;
    }

    // Send command to IIWA
    if (goalValid){
        positionCmdMsg.data.resize(goal.nbJoints);
        for (uint32_t i = 0; i < goal.nbJoints; ++i){
            positionCmdMsg.data[i] = goal.goalPose[i];
        }
        commandPub_.publish(positionCmdMsg);

        // Feedback loop
        
        std::unique_lock<std::mutex> jointSateLock(jointStateMtx_);
        jointStateLock.unlock();
        while (true){
            
            // Check if goal is reached
            jointStateLock.lock();
            currentError = jointState_ - goalJointPos;
            jointStateLock.unlock();
            goalReached = this->goalReached(currentError.get_position(), goal.tol);

            // Break out when goal is reached
            if(goalReached){ 
                break;
            }

            // Format and send feedback
            feedbackMsg.nbJoints = robotModel_->get_number_of_joints();
            feedbackMsg.currentError.resize(feedbackMsg.nbJoints);
            Eigen::VectorXd currentErrorVec = currentError.get_position();
            for (uint32_t i = 0; i < feedbackMsg.nbJoints; ++i){
                feedbackMsg.currentError[i] = currentErrorVec[i];
            }

            // Sleep until next loop
            FBRate.sleep();
        }
    }

    // Result message
    resultMsg.nbJoints = robotModel_->get_number_of_joints();
    resultMsg.positionReached = goalReached();
    jointStateLock.lock();
    currentError = jointState_ - goalJointPos;
    jointStateLock.unlock();
    Eigen::VectorXd currentErrorVec = currentError.get_position();
    resultMsg.currentError.resize(resultMsg.nbJoints);
    for (uint32_t i = 0; i < feedbackMsg.nbJoints; ++i){
        resultMsg.currentError[i] = currentErrorVec[i];
    }
}

bool goalReached(Eigen::VectorXd posDiff, std_msgs::Float64MultiArray tol)
{

    bool goalIsReached = true;
    posDiff = posDiff.cwiseAbs();

    for (uint32_t i = 0; i < posDiff.size(); ++i)
    {
        if (posDiff[i] > tol.data[i]){
            goalIsReached = false;
            break;
        }
    }
    return goalIsReached;
}
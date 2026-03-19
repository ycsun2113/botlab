#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.01f) // origin: 0.005f, 0.1f nice for 1000 particles, 0.01f for 1500 particles
, k2_(0.03f) // origin: 0.025f, 0.1f nice for 1000 particles, 0.03f for 1500 particles
// : k1_(0.005f)
// , k2_(0.025)
, min_dist_(0.0025) // origin: 0.0025
, min_theta_(0.02) // origin: 0.02
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    
    std::random_device rd;
    numberGenerator_ = std::mt19937(rd());

}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    if(!initialized_){
        resetPrevious(odometry);
        initialized_ = true;
    }

    dx_ = odometry.x - previousPose_.x;
    dy_ = odometry.y - previousPose_.y;
    // dtheta_ = odometry.theta - previousPose_.theta;
    dtheta_ = angle_diff(odometry.theta, previousPose_.theta);

    trans_ = std::sqrt(dx_*dx_ + dy_*dy_);
    rot1_ = angle_diff(std::atan2(dy_, dx_), previousPose_.theta);

    double direction = 1.0;
    if(std::abs(rot1_) > (M_PI)){
        rot1_ = angle_diff(M_PI, rot1_);
        direction = -1.0;
    }

    rot2_ = angle_diff(dtheta_, rot1_);
     
    moved_ = (dx_ != 0.0) || (dy_ != 0.0) || (dtheta_ != 0.0);

    if(std::fabs(trans_) < min_dist_ || std::fabs(dtheta_) < min_theta_){
        moved_ = false;
    }

    if(moved_){
        // rot1Std_ = std::sqrt(k1_ * std::abs(rot1_));
        // transStd_ = std::sqrt(k2_ * std::abs(trans_));
        // rot2Std_ = std::sqrt(k1_ * std::abs(rot2_));
        // rot1Std_ = std::sqrt(k1_ * std::abs(rot1_) * std::abs(rot1_));
        // transStd_ = std::sqrt(k2_ * std::abs(trans_) * std::abs(trans_));
        // rot2Std_ = std::sqrt(k1_ * std::abs(rot2_) * std::abs(rot2_));
        rot1Std_ = (k1_ * std::abs(rot1_) * std::abs(rot1_));
        transStd_ = (k2_ * std::abs(trans_) * std::abs(trans_));
        rot2Std_ = (k1_ * std::abs(rot2_) * std::abs(rot2_));
    }
    else{
        rot1Std_ = 0.0;
        transStd_ = 0.0;
        rot2Std_ = 0.0;
    }

    trans_ *= direction;
    previousPose_ = odometry;
    utime_ = odometry.utime;

    return moved_;

    // return false;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample = sample;
    
    double sampleRot1 = std::normal_distribution<double>(0.0, rot1Std_)(numberGenerator_);
    double sampleTrans = std::normal_distribution<double>(0.0, transStd_)(numberGenerator_);
    double sampleRot2 = std::normal_distribution<double>(0.0, rot2Std_)(numberGenerator_);

    double rot1_hat = angle_diff(rot1_, sampleRot1);
    double trans_hat = trans_ - sampleTrans;
    double rot2_hat = angle_diff(rot2_, sampleRot2);

    newSample.pose.x += trans_hat * cos(sample.pose.theta + rot1_hat);
    newSample.pose.y += trans_hat * sin(sample.pose.theta + rot1_hat);
    newSample.pose.theta  = wrap_to_pi(sample.pose.theta + rot1_hat + rot2_hat);

    newSample.pose.utime = utime_;
    newSample.parent_pose = sample.pose;

    return newSample;
}

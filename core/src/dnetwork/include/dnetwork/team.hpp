/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file team.hpp
 * @brief
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#pragma once
#include "dconfig/dconstant.hpp"
#include <dprocess/dprocess.hpp>
#include <dtransmit/dtransmit.hpp>

#include <dmsgs/BehaviorInfo.h>
#include <dmsgs/GCInfo.h>
#include <dmsgs/MotionInfo.h>
#include <dmsgs/TeamInfo.h>
#include <dmsgs/VisionInfo.h>

#include <mutex>
#include <ros/ros.h>
#include <string>

namespace dnetwork {

/**
 * @brief Sending and receiving robot information through DTransmit
 */
class Team : public dprocess::DProcess<Team>
{
  public:
    /**
     * @brief Team constructor
     *
     * @param nh - node handler
     */
    explicit Team(ros::NodeHandle* nh);
    /**
     * @brief Team destructor
     */
    ~Team();
    /**
     * @brief Tick
     */
    void tick() override;

  private:
    //! Node handler
    ros::NodeHandle* nh_;
    //! Subscriber for motion topic
    ros::Subscriber motion_sub_;
    //! Subscriber for behavior topic
    ros::Subscriber behavior_sub_;
    //! Subscriber for vision topic
    ros::Subscriber vision_sub_;
    //! Subscriber for game controller topic
    ros::Subscriber gc_sub_;

    //! Publisher for team topic
    ros::Publisher pub_;

    //! Transmitter for receiving and sending TeamInfo through UDP
    dtransmit::DTransmit* transmitter_;
    //! UDP boardcast address
    std::string udp_broadcast_address;
    //! Lock for data
    std::mutex data_lock_;
    //! Lock for data
    std::mutex info_lock_;
    //! TeamInfo
    dmsgs::TeamInfo info_;
    int test;
    //! Robot ID for current player
    int player_number_;
    //! Team ID for current team
    int team_number_;
    // TODO what's the meaning of cyan? Goal is on the left-half or right-half field?
    //! Flag for whether or not current team is cyan
    bool team_cyan_;

    //! Flag for whether or not robot is penalised
    bool penalised_ = false;
    //! Flag for whether or not robot is unstable
    bool unstable_ = true;

    //! Flag for whether or not vision info is ready
    bool visionReady_ = false;
    //! Flag for whether or not behavior info is ready
    bool behaviorReady_ = false;
    //! Flag for whether or not motion info is ready
    bool motionReady_ = false;

    /**
     * @brief Callback function for motion message 
     *
     * @param msg - received motion message
     */
    void MotionCallback(const dmsgs::MotionInfo::ConstPtr& msg);
    /**
     * @brief Callback function for behavior message 
     *
     * @param msg - received behavior message
     */
    void BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr& msg);
    /**
     * @brief Callback function for vision message 
     *
     * @param msg - received vision message
     */
    void VisionCallback(const dmsgs::VisionInfo::ConstPtr& msg);
    /**
     * @brief Callback function for game controller message 
     *
     * @param msg - received game controller message
     */
    void GCCallback(const dmsgs::GCInfo::ConstPtr& msg);
};

} // namespace dnetwork

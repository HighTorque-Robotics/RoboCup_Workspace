/* Copyright (C) ZJUDancer
 * 2017 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file team.cpp
 * @brief
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

#include "dnetwork/team.hpp"
#include <dmsgs/GCInfo.h>

namespace dnetwork {

static const int NETWORK_FREQ = 30;
Team::Team(ros::NodeHandle *nh) : DProcess(NETWORK_FREQ, false), nh_(nh) {
  // get params from config server
  if (!nh_->getParam("RobotId", player_number_))
    throw std::runtime_error("Can't get player number!");

  if (!nh_->getParam("/ZJUDancer/TeamNumber", team_number_))
    throw std::runtime_error("Can't get team number!");

  if (!nh_->getParam("/ZJUDancer/TeamCyan", team_cyan_))
    throw std::runtime_error("Can't decide team color!");

  if (!nh_->getParam("/ZJUDancer/udpBroadcastAddress", udp_broadcast_address))
    throw std::runtime_error("Can't get udp broadcast address!");

  if (!nh_->getParam("TEST", test))
    throw std::runtime_error("Can't get test");

  // ROS subscriber and publisher
  motion_sub_ = nh_->subscribe("/dmotion_" + std::to_string(player_number_) +
                                   "/MotionInfo",
                               1, &Team::MotionCallback, this);
  vision_sub_ = nh_->subscribe("/dvision_" + std::to_string(player_number_) +
                                   "/VisionInfo",
                               1, &Team::VisionCallback, this);
  behavior_sub_ = nh_->subscribe(
      "/dbehavior_" + std::to_string(player_number_) + "/BehaviorInfo", 1,
      &Team::BehaviorCallback, this);
  gc_sub_ =
      nh_->subscribe("/dnetwork_" + std::to_string(player_number_) + "/GCInfo",
                     1, &Team::GCCallback, this);
  pub_ = nh_->advertise<dmsgs::TeamInfo>(
      "/dnetwork_" + std::to_string(player_number_) + "/TeamInfo", 1);

  transmitter_ = new dtransmit::DTransmit();
  transmitter_->addRawRecv(dconstant::network::TeamInfoBroadcastAddress, [this](void *buffer, std::size_t size) {
    if (size == sizeof(dmsgs::TeamInfo)) {
      std::unique_lock<std::mutex> lock(data_lock_);
      dmsgs::TeamInfo team_info =
          *(dmsgs::TeamInfo *)buffer;
      // if (true) {
      if (team_info.team_number == team_number_) {
      // if (team_info.player_number != player_number_ && team_info.team_number == team_number_) {
        team_info.recv_timestamp = ros::Time::now();
        // ROS_INFO("Heard message from robot %d in team %d\n", team_info.player_number, team_info.team_number);
        pub_.publish(team_info);
      }
    }
  });
  transmitter_->startService();
}

Team::~Team() {}

void Team::tick() {
  info_.player_number = player_number_;
  info_.incapacitated = false;
  info_.team_number = team_number_;

  if (unstable_) {
    // std::cout << (int)unstable_ << " " << (int)penalised_ << std::endl;
    info_.incapacitated = true;
  }

  // TODO add lock for message receiving and sending
  // if (motionReady_ && visionReady_ && behaviorReady_) {
  if (behaviorReady_) {
  // if (true) {
    info_.txp_timestamp = ros::Time::now();
    transmitter_->sendRaw(dconstant::network::TeamInfoBroadcastAddress,
                          (void *)&info_, sizeof(info_));
    motionReady_ = false;
    visionReady_ = false;
    behaviorReady_ = false;
  }
  // ROS_INFO("team info is sent");
}

void Team::MotionCallback(const dmsgs::MotionInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::MotionInfo motion_info = *msg;
  unstable_ = !motion_info.stable;
  // ROS_INFO("motion info ready");
  motionReady_ = true;
}

void Team::BehaviorCallback(const dmsgs::BehaviorInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::BehaviorInfo behavior_info = *msg;
  info_.role = behavior_info.current_role;
  info_.attack_right = behavior_info.attack_right;

  info_.state = behavior_info.team_play_state;
  info_.kicker_id = behavior_info.kicker_id;
  info_.priority = behavior_info.team_play_priority;
  info_.mates_online = behavior_info.mates_online;

  info_.dest = behavior_info.dest;
  info_.final_dest = behavior_info.final_dest;
  info_.time_since_last_kick = behavior_info.time_since_last_kick;
  info_.attack_target = behavior_info.attack_target;
  // ROS_INFO("behavior info ready");
  behaviorReady_ = true;
}

void Team::VisionCallback(const dmsgs::VisionInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::VisionInfo vision_info = *msg;
  info_.see_ball = vision_info.see_ball;
  info_.see_circle = vision_info.see_circle;
  info_.see_goal = vision_info.see_goal;
  info_.robot_pos = vision_info.robot_pos;

  info_.ball_field = vision_info.ball_field;
  info_.ball_global = vision_info.ball_global;

  info_.circle_field = vision_info.circle_field;
  info_.circle_global = vision_info.circle_global;

  info_.goal_field = vision_info.goal_field;
  info_.goal_global = vision_info.goal_global;

  info_.ball_quality = vision_info.ball_quality;
  info_.field_quality = vision_info.field_quality;
  info_.field_consistency = vision_info.field_consistency;
  visionReady_ = true;
}

void Team::GCCallback(const dmsgs::GCInfo::ConstPtr &msg) {
  std::lock_guard<std::mutex> lock(info_lock_);
  dmsgs::GCInfo gc_info = *msg;
  penalised_ = gc_info.penalised;
  info_.gc_connected = gc_info.connected;
  info_.gc_state = gc_info.state;
  info_.gc_state2 = gc_info.secondaryState;
}

} // namespace dnetwork

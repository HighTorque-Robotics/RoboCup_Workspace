/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * \file dvision.cpp
 * \brief Main class of vision module
 * \author Yusu Pan, Wenxing Mei
 * \version 2018
 * \date 2018-02-12
 */

#include "dvision/dvision.hpp"

using namespace dancer_geometry;

namespace dvision {
DVision::DVision()
    : profiler_step_1_("Step 1"),
      ball_quality_(0),
      profiler_step_2_("Step 2"),
      profiler_others_("Others") {
  // Initialize various member
  ROS_INFO("[dvision] starting");
  nh_ = new ros::NodeHandle("~");

  parameters.init(nh_);
  projection_.init(nh_);
  projection2_.init(nh_);
  amcl_.Init();
  buffer_gui_image_.Init(4, 30);
  buffer_info_.Init(2, 30);
  tracker_.Init(parameters.camera.extrinsic_para, parameters.camera.fx,
                parameters.camera.fy, parameters.camera.undistCx,
                parameters.camera.undistCy, parameters.camera.centerInUndistX,
                parameters.camera.centerInUndistY);
  if (parameters.ball.velocityDebug) {
    log_file_.open("/home/daiz/dancer-debug/ball.txt");
    log_file_ << "time, rawx, rawy, kalmanx, kalmany, vx, vy\n";
  }
  // TODO check what's unnecessary in simulation mode and remove
  if (!parameters.simulation) {
    obstacle_.Init();
    ball_.Init();
        

    circle_.Init();
    corner_.Init();
    goal_.Init();
    line_classifier_.Init();
    obj_.Init();
    
    ROS_INFO("detect init success");
  }

  if (parameters.offline_record) {
    image_transport::ImageTransport pub(*nh_);
    pub_image_ = pub.advertise("cam_image", 1, this);
    ROS_WARN("[DVision] Recording mode");
  } else if (parameters.offline_replay) {
    image_transport::ImageTransport sub(*nh_);
    sub_image_ = sub.subscribe(
        "/dvision_" + std::to_string(parameters.robotId) + "/cam_image", 1,
        &DVision::imageCallback, this);
    ROS_WARN("[DVision] Replaying mode");
  }

  ROS_INFO("[dvision] undist %lf %lf", parameters.camera.centerInUndistX,
           parameters.camera.centerInUndistY);

  // Frame::initEncoder();

  InitVisionYaw();

  // Initialize receiver and transmitter
  sub_motion_info_ = nh_->subscribe(
      "/dmotion_" + std::to_string(parameters.robotId) + "/MotionInfo", 1,
      &DVision::MotionCallback, this);
  sub_behavior_info_ = nh_->subscribe(
      "/dbehavior_" + std::to_string(parameters.robotId) + "/BehaviorInfo", 1,
      &DVision::BehaviorCallback, this);
  sub_reload_config_ = nh_->subscribe("/humanoid/ReloadVisionConfig", 1,
                                      &DVision::ReloadConfigCallback, this);
  pub_ = nh_->advertise<VisionInfo>("VisionInfo", 1);
  //    pub_debug_ = nh_->advertise<VisionDebugInfo>("VisionDebugInfo", 1);
  serv_toggle_amcl_ =
      nh_->advertiseService("toggle_amcl", &DVision::toggleAMCL, this);
  serv_reset_particle_touch_line_ = nh_->advertiseService(
      "reset_particles_touch_line", &DVision::ResetParticlesTouchLine, this);
  serv_reset_particle_left_touch_ = nh_->advertiseService(
      "reset_particles_left_touch", &DVision::ResetParticlesLeftTouch, this);
  serv_reset_article_right_touch_ = nh_->advertiseService(
      "reset_particles_right_touch", &DVision::ResetParticlesRightTouch, this);
  serv_reset_particle_point_ = nh_->advertiseService(
      "reset_particles_point", &DVision::ResetParticlesPoint, this);
#ifdef USE_NEO_INFO
  serv_reset_yaw_ =
      nh_->advertiseService("reset_yaw", &DVision::SetInitOrientation, this);
#endif

  // Simulation or Not
  if (parameters.simulation) {
    ROS_WARN("[dvision] now in SIMULATION mode");
    // init transmitter in local
    // transmitter_ = new dtransmit::DTransmit();
    // // receive and update vision info and vision yaw from sim server
    // transmitter_->addRosRecv<VisionInfo>(
    //     dconstant::network::monitorBroadcastAddressBase + parameters.robotId,
    //     [&](VisionInfo &msg) {
    //       m_sim_vision_info = msg;
    //       vision_yaw_ = msg.simYaw / 180.0 * M_PI;
    //     });
    // transmitter_->startService();
  } else {
    if (parameters.vision_only_mode) {
      ROS_WARN("[dvision] now in VISION_ONLY mode");
    } else {
      ROS_WARN("[dvision] now in REALWORLD mode");
    }
    // init camera with settings
    if (!parameters.offline_replay) {
      CameraSettings s(nh_);
#ifdef USE_GST_CAMERA
      camera_ = new GstCamera(s.device, s.width, s.height);
#else
      camera_ = new V4L2Camera(s);
#endif
    }

    // open debug window if needed
    //    if (parameters.camera.debug) {
    //      camera_->enableDebugWindow();
    //    }

    // init transmitter for broadcast
    if (parameters.broadcast_vision_info) {
      transmitter_ = new dtransmit::DTransmit();
      transmitter_->startService();
    }
  }

  // Setup concurrent
  if (!parameters.simulation) {
    // detect objects like ball or goal
    concurrent_.push([&]() {
#ifdef USE_GST_CAMERA
      obj_.Detect(frame_.bgr_cuda(), workspace_->guiImg, ball_quality_);
#else
      obj_.Detect(frame_.bgr(), workspace_->guiImg, ball_quality_);
#endif
      workspace_->ballPosition = obj_.ball_position();
      workspace_->goalPosition = obj_.goal_position();
      workspace_->robotPosition = obj_.obstacle_position();
      workspace_->fieldFeatures = obj_.field_features();
    });
  } else {
    // detect nothing, just receive landmark info
    concurrent_.push([&]() {
      workspace_->visionInfo = m_sim_vision_info;
      workspace_->visionInfo.behaviorInfo = m_behavior_info;
    });
  }
}

DVision::~DVision() {
  std::cout << profiler_step_1_ << std::endl;
  std::cout << profiler_step_2_ << std::endl;
  std::cout << profiler_others_ << std::endl;
  delete camera_;
  delete transmitter_;
}

void DVision::Start() {
  // run step 1 at 30fps
  t1_ = std::thread([&]() {
    ros::Rate r(30);
    while (ros::ok()) {
      ros::spinOnce();
      step1();
      r.sleep();
    }
  });

  // run step 2 at 30fps
  t2_ = std::thread([&]() {
    ros::Rate r(30);
    while (ros::ok()) {
      step2();
      r.sleep();
    }
  });
}

void DVision::Join() {
  t1_.join();
  t2_.join();
}

void DVision::step1() {
  profiler_step_1_.StartTimer();
  CheckMotionConnection();
  workspace_ = &buffer_info_.WorkerRequest();
  profiler_step_1_.LogAndRestartTimer("[1] request buffer");

  // capture frame from camera
  if (!parameters.simulation &&
      (!parameters.offline_replay || parameters.offline_record)) {
    frame_ = camera_->capture();
    profiler_step_1_.LogAndRestartTimer("[1] capture frame");
  }

  // if stable, do vision task
  // FIXME may need mutex for motion_stable_
  if (!parameters.offline_record && parameters.offline_replay) {
    workspace_->visionInfo = VisionInfo();
    workspace_->guiImg = frame_.bgr();
    workspace_->hsvImg = frame_.hsv();
    workspace_->pitch = plat_pitch_;
    workspace_->yaw = plat_yaw_;
    projection_.updateExtrinsic(plat_pitch_, plat_yaw_);
    profiler_step_1_.LogAndRestartTimer("[1] update extrinsic");

    if (!workspace_->guiImg.empty()) {
      // detect objs, field and obstacles
      concurrent_.spinOnce();
      concurrent_.join();
      profiler_step_1_.LogAndRestartTimer(
          "[1] detect field & objs & obstacles");
    }
  } else if (parameters.vision_only_mode or
             (motion_connected_ && motion_stable_ && plat_pitch_ >= 0)) {
    // update pitch and yaw, then update camera extrinsic parameters
    workspace_->pitch = plat_pitch_;
    workspace_->yaw = plat_yaw_;
    projection_.updateExtrinsic(plat_pitch_, plat_yaw_);
    profiler_step_1_.LogAndRestartTimer("[1] update extrinsic");

    // update images from frame
    if (!parameters.simulation) {
      workspace_->visionInfo = VisionInfo();
      workspace_->guiImg = frame_.bgr_raw();
      // cvtColor(workspace_->guiImg, workspace_->grayImg, CV_BGR2GRAY); // -2.5
      workspace_->hsvImg = frame_.hsv();
      profiler_step_1_.LogAndRestartTimer("[1] convert frame");
      if (parameters.offline_record) {
        std_msgs::Header header;
        cv_bridge::CvImage cvmsg(header, "bgr8", frame_.bgr().clone());
        sensor_msgs::ImagePtr msg = cvmsg.toImageMsg();
        pub_image_.publish(msg);
        profiler_step_1_.LogAndRestartTimer("[1] publish frame");
      }
    }

    // detect objs, field and obstacles
    concurrent_.spinOnce();
    concurrent_.join();
    profiler_step_1_.LogAndRestartTimer("[1] detect field & objs & obstacles");
  }
  buffer_info_.WorkerRelease();
  profiler_step_1_.LogAndRestartTimer("[1] release buffer");
}

void DVision::step2() {
  profiler_step_2_.StartTimer();
  auto &visionSharedInfo = buffer_info_.UserRequest();
  profiler_step_2_.LogAndRestartTimer("[2] request buffer");

  ROS_DEBUG("[dvision] motion_connected_: %d motionStable: %d, pitch: %lf",
            (int)motion_connected_, (int)motion_stable_,
            visionSharedInfo.pitch);

  if (parameters.vision_only_mode or
      (motion_connected_ && motion_stable_ && visionSharedInfo.pitch >= 0)) {
    // update camera extrinsic parameters based on pitch and yaw in shared info
    // std::cout<<"pitch"<<visionSharedInfo.pitch<<"---yaw"<<visionSharedInfo.yaw<<std::endl;
    projection2_.updateExtrinsic(visionSharedInfo.pitch, visionSharedInfo.yaw);
    profiler_step_2_.LogAndRestartTimer("[2] update extrinsic");

    Measurement::LandMark mark;
    measurement_ = Measurement();
    // Detection
    if (!parameters.simulation) {
      goal_.Detect(visionSharedInfo.goalPosition, projection2_,
                   visionSharedInfo.visionInfo, delta_, measurement_);
      profiler_step_2_.LogAndRestartTimer("[2] detect goal");

      corner_.Detect(visionSharedInfo.fieldFeatures, projection2_,
                     visionSharedInfo.visionInfo, visionSharedInfo.guiImg,
                     measurement_);
      profiler_step_2_.LogAndRestartTimer("[2] detect corner");

      circle_.Detect(corner_.getXIntx(), projection2_,
                     visionSharedInfo.visionInfo, delta_, measurement_);
      profiler_step_2_.LogAndRestartTimer("[2] detect circle");

      ball_.Detect(visionSharedInfo.ballPosition, visionSharedInfo.guiImg,
                   visionSharedInfo.fieldHullReal,
                   visionSharedInfo.fieldBinaryRaw, projection2_,
                   visionSharedInfo.visionInfo, delta_);
      profiler_step_2_.LogAndRestartTimer("[2] detect ball");

      ShowDebugImg(visionSharedInfo);
      profiler_step_2_.LogAndRestartTimer("[2] show debug image");
    }

    // Correct vision yaw (a.k.a field angle)
    // Protect vision yaw and delta
    std::unique_lock<std::mutex> lk(lock_vision_yaw_);
    //        VisionDebugInfo debug_info;
    //        debug_info.yaw_raw = vision_yaw_;
    projection2_.UpdateHeadingOffset(vision_yaw_);
    auto delta = delta_;
    auto yaw = vision_yaw_;
    //        debug_info.yaw_corrected = vision_yaw_;
    //        debug_info.yaw_bias = debug_info.yaw_corrected -
    //        debug_info.yaw_raw;
    lk.unlock();
    profiler_step_2_.LogAndRestartTimer("[2] correct vision yaw");

    // Localization

    // TODO add gaussian noise in AMCL when falled down
    //        if(fallen_) {
    //            amcl_.falldownGauss();
    //            fallen_ = false;
    //       }

    // self-localization by AMCL and measurements
    amcl_.Process(measurement_, delta, yaw / M_PI * 180.0,
                  visionSharedInfo.visionInfo, crouching_);
    // add quality
    float field_quality = amcl_.GetQuality(),
          field_consistency = amcl_.GetConsistency();
    visionSharedInfo.visionInfo.ball_quality = ball_quality_;
    visionSharedInfo.visionInfo.field_quality = field_quality;
    visionSharedInfo.visionInfo.field_consistency = field_consistency;
    ROS_DEBUG("[dvision] ball quality: %f", ball_quality_);
    ROS_DEBUG("[dvision] field quality: %f", field_quality);
    ROS_DEBUG("[dvision] field consistency: %lf", field_consistency);
    profiler_step_2_.LogAndRestartTimer("[2] localization");

    UpdateViewRange(visionSharedInfo.visionInfo);
    profiler_step_2_.LogAndRestartTimer("[2] update view range");

    // Track Ball
    // Calc global position of ball
    cv::Point2f ball_field(visionSharedInfo.visionInfo.ball_field.x,
                           visionSharedInfo.visionInfo.ball_field.y);
    cv::Point2f ball_global =
        getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos,
                              visionSharedInfo.visionInfo.ball_field);
    visionSharedInfo.visionInfo.ball_global.x = ball_global.x;
    visionSharedInfo.visionInfo.ball_global.y = ball_global.y;
    // compute desired pitch and yaw to track ball
    if (visionSharedInfo.visionInfo.see_ball) {
      if (tracker_.Process(visionSharedInfo.visionInfo.ball_field.x,
                           visionSharedInfo.visionInfo.ball_field.y)) {
        visionSharedInfo.visionInfo.ballTrack.pitch =
            Radian2Degree(tracker_.out_pitch());
        visionSharedInfo.visionInfo.ballTrack.yaw =
            Radian2Degree(tracker_.out_yaw());
      }
      profiler_step_2_.LogAndRestartTimer("[2] track ball");
    }

    // Track circle
    // auto &r = visionSharedInfo.visionInfo.robot_pos;
    // auto robotPos = cv::Point3d(r.x, r.y, r.z);
    // auto circleField = getOnRobotCoordinate(robotPos, cv::Point2f(0, 0));
    // compute desired pitch and yaw to track center circle
    if (visionSharedInfo.visionInfo.see_circle) {
      auto circleField = visionSharedInfo.visionInfo.circle_field;
      if (tracker_.Process(circleField.x, circleField.y)) {
        visionSharedInfo.visionInfo.circleTrack.pitch =
            Radian2Degree(tracker_.out_pitch());
        visionSharedInfo.visionInfo.circleTrack.yaw =
            Radian2Degree(tracker_.out_yaw());
      }
      profiler_step_2_.LogAndRestartTimer("[2] track circle");
    }
    // Calc global position of obstacles
    for (auto &g : visionSharedInfo.visionInfo.obstacles_field) {
      cv::Point2f obstacle_global =
          getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos, g);
      geometry_msgs::Vector3 tmp_obs;
      tmp_obs.x = obstacle_global.x;
      tmp_obs.y = obstacle_global.y;
      visionSharedInfo.visionInfo.obstacles_global.push_back(tmp_obs);
    }

    // Track Goal
    if (visionSharedInfo.visionInfo.see_goal) {
      // Calc global position of goals
      cv::Point2f goal_global =
          getOnGlobalCoordinate(visionSharedInfo.visionInfo.robot_pos,
                                visionSharedInfo.visionInfo.goal_field);
      visionSharedInfo.visionInfo.goal_global.x = goal_global.x;
      visionSharedInfo.visionInfo.goal_global.y = goal_global.y;

      cv::Point2f goal_field;
      goal_field.x = visionSharedInfo.visionInfo.goal_field.x;
      goal_field.y = visionSharedInfo.visionInfo.goal_field.y;
      if (tracker_.Process(goal_field.x, goal_field.y)) {
        visionSharedInfo.visionInfo.goalTrack.pitch =
            Radian2Degree(tracker_.out_pitch());
        visionSharedInfo.visionInfo.goalTrack.yaw =
            Radian2Degree(tracker_.out_yaw());
      }
    }
    profiler_step_2_.LogAndRestartTimer("[2] track goal");

    // Update ball velocity
    if (motion_info_.status == dmsgs::MotionInfo::STANDBY &&
        visionSharedInfo.visionInfo.see_ball) {
      ball_pos_history_.push(BallPos(ball_field, ros::Time::now().toSec()));
      // Skip velocity calculating when history is not enough, which will cause
      // large noise
      if (ball_pos_history_.size() >= parameters.ball.minQueue) {
        // TODO(daiz): Add kalman for velocity?
        if (ball_pos_history_.size() > parameters.ball.maxQueue)
          ball_pos_history_.pop();
        cv::Point2f velocity;
        BallPos first = ball_pos_history_.front(),
                last = ball_pos_history_.back();
        double span = last.time - first.time;
        velocity.x = (last.pos.x - first.pos.x) / span;
        velocity.y = (last.pos.y - first.pos.y) / span;
        visionSharedInfo.visionInfo.ball_velocity.x = velocity.x;
        visionSharedInfo.visionInfo.ball_velocity.y = velocity.y;

        if (parameters.ball.velocityDebug) {
          // Write to file for debug
          std::string time = std::to_string(ros::Time::now().toSec());
          log_file_ << time << ", " << ball_field.x << ", " << ball_field.y
                    << ", " << ball_field.x << ", " << ball_field.y << ", "
                    << velocity.x << ", " << velocity.y << "\n";
        }
      }
    } else
      ball_pos_history_ = std::queue<BallPos>();
    profiler_step_2_.LogAndRestartTimer("[2] update ball velocity");

    // Broadcast vision info
    pub_.publish(visionSharedInfo.visionInfo);
    if (parameters.broadcast_vision_info && !(++cycle2_ % 3)) {
      transmitter_->sendRos(
          dconstant::network::robotBroadcastAddressBase + parameters.robotId,
          visionSharedInfo.visionInfo);
    }
    profiler_step_2_.LogAndRestartTimer("[2] pub vision info");
  }

  // Remember to release, otherwise dead lock..
  buffer_info_.UserRelease();
  profiler_step_2_.LogAndRestartTimer("[2] release buffer");

  // Cache gui image
  //  if (!parameters.simulation && parameters.monitor.update_gui_img) {
  //    auto &guiImg = buffer_gui_image_.WorkerRequest();
  //    guiImg = visionSharedInfo.guiImg.clone();
  //    buffer_gui_image_.WorkerRelease();
  //    profiler_step_2_.LogAndRestartTimer("[2] cache gui image");
  //  }
}

// TEST(MWX): #1
// Avoid correct heading in init
void DVision::InitVisionYaw() {
  plat_pitch_ = 0;
  plat_yaw_ = 0;
  imu_roll_ = 0;
  imu_pitch_ = 0;
  imu_yaw_pre_ = M_PI / 2;
  imu_yaw_cur_ = M_PI / 2;
  imu_yaw_delta_ = 0;
  vision_yaw_ = M_PI / 2;
  vision_x_pre_ = 0;
  vision_y_pre_ = 0;
  vision_x_cur_ = 0;
  vision_y_cur_ = 0;
  vision_x_delta_ = 0;
  vision_y_delta_ = 0;
  projection2_.ClearHeadingOffsetBias();
}

bool DVision::CheckMotionConnection() {
  auto now = ros::Time::now();
  auto elapsedSec = now.toSec() - time_last_motion_recv.toSec();
  auto conn_elapsedSec = now.toSec() - time_last_motion_conn.toSec();
  motion_connected_ =
      ((elapsedSec < 1) && (conn_elapsedSec < 1)) || parameters.simulation;
  return motion_connected_;
}

void DVision::MotionCallback(const dmsgs::MotionInfo::ConstPtr &motion_msg) {
  profiler_others_.StartTimer();
  time_last_motion_recv = ros::Time::now();
  motion_info_ = *motion_msg;
  crouching_ = false;
  if (motion_info_.lower_board_connected)
    time_last_motion_conn = ros::Time::now();

  std::lock_guard<std::mutex> lk(lock_vision_yaw_);

  // init vision yaw (aka field angle) when reconnected
  auto now = ros::Time::now();
  auto conn_elapsedSec = now.toSec() - time_last_motion_conn.toSec();
  if (conn_elapsedSec < 1 && !motion_connected_) {
    ROS_FATAL("Motion reconnected, init vision_yaw");
    InitVisionYaw();
    dx_sum_ = 0;
    dy_sum_ = 0;
    dt_sum_ = 0;
    imu_inited_ = false;
  }

  // update motion info at present
  motion_stable_ = motion_info_.stable;
#ifdef USE_NEO_INFO
  plat_pitch_ = motion_info_.curPlat.pitch;
  plat_yaw_ = motion_info_.curPlat.yaw;
#else
  plat_pitch_ = motion_info_.action.headCmd.pitch;
  plat_yaw_ = motion_info_.action.headCmd.yaw;
#endif
  imu_roll_ = motion_info_.imuRPY.x;
  imu_pitch_ = motion_info_.imuRPY.y;
  // backup previous IMU info and accumulate vision yaw
  if (!imu_inited_) {
    imu_yaw_pre_ = motion_info_.imuRPY.z;
    imu_yaw_cur_ = motion_info_.imuRPY.z;
#ifdef USE_NEO_INFO
    previous_delta_ = motion_info_.odometry;
#else
    previous_delta_ = motion_info_.deltaData;
#endif
    imu_inited_ = true;
  } else if (motion_stable_) {
    imu_yaw_pre_ = imu_yaw_cur_;
    imu_yaw_cur_ = motion_info_.imuRPY.z;
#ifdef USE_NEO_INFO
    imu_yaw_delta_ = imu_yaw_cur_ - imu_yaw_pre_;
    vision_yaw_ += Degree2Radian(imu_yaw_delta_);
#else
    if (parameters.simulation) {
      // imu_yaw_delta_ = imu_yaw_cur_ - imu_yaw_pre_;
      // vision_yaw_ += imu_yaw_delta_;
    } else {
      imu_yaw_delta_ = imu_yaw_cur_ - imu_yaw_pre_;
      // TODO check it's degree or radian
      vision_yaw_ += imu_yaw_delta_;
    }
#endif
  }

  // restrict vision yaw at (-90, 90) degree
  if (vision_yaw_ > M_PI) {
    vision_yaw_ -= 2 * M_PI;
  } else if (vision_yaw_ <= -M_PI) {
    vision_yaw_ += 2 * M_PI;
  }

  // correct vision yaw
  {
#ifdef USE_NEO_INFO
    auto tmpDelta = motion_info_.odometry;
#else
    auto tmpDelta = motion_info_.deltaData;
#endif
    auto dx = tmpDelta.x - previous_delta_.x;
    auto dy = tmpDelta.y - previous_delta_.y;
    auto dt = tmpDelta.z - previous_delta_.z;

#ifdef USE_NEO_INFO
    if (!motion_stable_) {
      dx = 0;
      dy = 0;
    }
#else
    auto t = Degree2Radian(previous_delta_.z);
    auto ddx = dx * cos(-t) - dy * sin(-t);
    auto ddy = dx * sin(-t) + dy * cos(-t);

    vision_x_pre_ = vision_x_cur_;
    vision_y_pre_ = vision_y_cur_;

    vision_x_cur_ += ddx * cos(vision_yaw_) - ddy * sin(vision_yaw_);
    vision_y_cur_ += ddx * sin(vision_yaw_) + ddy * cos(vision_yaw_);

    if (!motion_stable_) {
      vision_x_delta_ = 0;
      vision_y_delta_ = 0;
    } else {
      vision_x_delta_ = vision_x_cur_ - vision_x_pre_;
      vision_y_delta_ = vision_y_cur_ - vision_y_pre_;
    }
#endif

    previous_delta_ = tmpDelta;

#ifdef USE_NEO_INFO
    delta_.dx = dx;
    delta_.dy = dy;
    delta_.dt = dt;
    bool start_kick = false;
    if (ros::Time::now().toSec() - time_start_kick_.toSec() > 10 &&
        motion_info_.status == dmsgs::MotionInfo::KICKING) {
      time_start_kick_ = ros::Time::now();
    }
    if (ros::Time::now().toSec() - time_start_kick_.toSec() < 2)
      start_kick = true;
    crouching_ =
        (motion_info_.status == dmsgs::MotionInfo::STANDBY || start_kick);
#else
    if (parameters.simulation) {
      delta_.dx = ddx;
      delta_.dy = ddy;
      delta_.dt = dt;
    } else {
      delta_.dx = vision_x_delta_;
      delta_.dy = vision_y_delta_;
      delta_.dt = dt;
    }
#endif

    if (parameters.calib.calibOdometer) {
      dx_sum_ += dx;
      dy_sum_ += dy;
      dt_sum_ += dt;
      ROS_INFO("[dvision] dx_sum_: %lf", dx_sum_);
      ROS_INFO("[dvision] dy_sum_: %lf", dy_sum_);
      ROS_INFO("[dvision] dt_sum_: %lf", dt_sum_);
    }
  }

  profiler_others_.LogAndRestartTimer("[others] motion callback");
}

void DVision::BehaviorCallback(
    const dmsgs::BehaviorInfo::ConstPtr &behavior_msg) {
  m_behavior_info = *behavior_msg;

  // prepare output directory
  std::string path(std::string(std::getenv("HOME")) + "/dancer-camera" + "/hum_data/");
  boost::filesystem::path dir(path.c_str());
  if (boost::filesystem::create_directory(dir)) {
    ROS_INFO("Directory Created for captured frames: %s", path.c_str());
  }

  // save image signal
  if (!parameters.simulation && m_behavior_info.save_image && !save_data_) {
    save_data_ = true;

    // capture current frame
    auto frame = camera_->capture();

    // save current frame
    std::string filename = path + std::to_string(frame.time_stamp().toNSec()) + ".jpg";
    frame.save(filename);
  }

  save_data_ = m_behavior_info.save_image;
}

void DVision::ReloadConfigCallback(const std_msgs::String::ConstPtr &) {
  parameters.update();
}

void DVision::imageCallback(const sensor_msgs::ImageConstPtr &msg) {
  cv::Mat img;
  img = cv_bridge::toCvShare(msg, "bgr8")->image;
  frame_.offlineUpdate(img);
}

void DVision::UpdateViewRange(VisionInfo &info) {
  cv::Point2f upperLeft;
  cv::Point2f upperRight;
  cv::Point2f lowerLeft;
  cv::Point2f lowerRight;

  projection2_.getOnRealCoordinate(cv::Point(0, 0), upperLeft);
  projection2_.getOnRealCoordinate(cv::Point(parameters.camera.width - 1, 0),
                                   upperRight);
  projection2_.getOnRealCoordinate(cv::Point(0, parameters.camera.height - 1),
                                   lowerLeft);
  projection2_.getOnRealCoordinate(
      cv::Point(parameters.camera.width - 1, parameters.camera.height - 1),
      lowerRight);

  auto v1 = upperLeft - lowerLeft;
  auto v2 = lowerRight - lowerLeft;
  auto theta = getAngleBetweenVectors(v1, v2);
  if (theta < 180 && theta > 0) {
    upperLeft *= -100;
  }

  v1 = upperRight - lowerRight;
  v2 = -v2;
  theta = getAngleBetweenVectors(v2, v1);
  if (theta < 180 && theta > 0) {
    upperRight *= -100;
  }

  info.viewRange.resize(4);
  info.viewRange[0].x = upperLeft.x;
  info.viewRange[0].y = upperLeft.y;
  info.viewRange[1].x = upperRight.x;
  info.viewRange[1].y = upperRight.y;
  info.viewRange[2].x = lowerRight.x;
  info.viewRange[2].y = lowerRight.y;
  info.viewRange[3].x = lowerLeft.x;
  info.viewRange[3].y = lowerLeft.y;
}

void DVision::ShowDebugImg(VisionSharedInfo &info) {
  if (parameters.simulation) return;

  if (parameters.monitor.use_cv_show) {
    if (info.hsvImg.cols != 0 && info.hsvImg.rows != 0) {
      // show gui image of detections
      if (parameters.monitor.update_gui_img) {
        gui_cnt_++;
        if (gui_cnt_ % 10 == 0) {
          cv::namedWindow("gui", CV_WINDOW_NORMAL);
          cv::imshow("gui", info.guiImg);
          gui_cnt_ = 0;
        }
      }
      cv::waitKey(1);
    }
  }
}

bool DVision::toggleAMCL(dmsgs::ToggleAMCL::Request &req,
                         dmsgs::ToggleAMCL::Response &res) {
  enable_amcl_ = (bool)req.swtich;
  ROS_DEBUG("[dvision] toggle amcl %s", (bool)req.swtich ? "true" : "false");
  return true;
}

bool DVision::ResetParticlesTouchLine(
    dmsgs::ResetParticleTouchLine::Request &req,
    dmsgs::ResetParticleTouchLine::Response &res) {
  auto &side = req.side;
  ROS_DEBUG("[dvision] reset particles touch line");
  amcl_.ResetParticlesTouchLine(side);
  return true;
}

bool DVision::ResetParticlesLeftTouch(
    dmsgs::ResetParticleLeftTouch::Request &req,
    dmsgs::ResetParticleLeftTouch::Response &res) {
  ROS_DEBUG("[dvision] reset particles left touch");
  amcl_.ResetParticlesLeftTouch();
  return true;
}

bool DVision::ResetParticlesRightTouch(
    dmsgs::ResetParticleRightTouch::Request &req,
    dmsgs::ResetParticleRightTouch::Response &res) {
  ROS_DEBUG("[dvision] reset particles right touch");
  amcl_.ResetParticlesRightTouch();
  return true;
}

bool DVision::ResetParticlesPoint(dmsgs::ResetParticlePoint::Request &req,
                                  dmsgs::ResetParticlePoint::Response &res) {
  auto &point = req.point;
  vision_yaw_ = Degree2Radian(point.z);
  amcl_.ResetParticlesPoint(point);
  ROS_DEBUG("[dvision] reset particles at point %lf %lf %lf", point.x, point.y,
            point.z);
  return true;
}

bool DVision::ResetParticlesRandom(dmsgs::ResetParticleRandom::Request &req,
                                   dmsgs::ResetParticleRandom::Response &res) {
  amcl_.ResetParticlesRandom();
  ROS_DEBUG("[dvision] reset particles randomly");
  return true;
}

#ifdef USE_NEO_INFO
bool DVision::SetInitOrientation(dmsgs::SetInitOrientation::Request &req,
                                 dmsgs::SetInitOrientation::Response &res) {
  vision_yaw_ = Degree2Radian(req.yaw);
  amcl_.SetYaw(req.yaw);
  ROS_DEBUG("[dvision] reset init orientation");
  return true;
}
#endif

}  // namespace dvision

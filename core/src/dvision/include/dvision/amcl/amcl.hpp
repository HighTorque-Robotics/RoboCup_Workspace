/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file amcl.hpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-22
 */

#pragma once
#include "dmsgs/ResetParticleTouchLine.h"
#include "dmsgs/VisionInfo.h"
#include "dvision/amcl/kdtree.hpp"
#include "dvision/amcl/map.hpp"
#include "dvision/amcl/particle.hpp"
#include "dvision/amcl/tools.hpp"
#include "dvision/amcl/types.hpp"

#include "geometry_msgs/Vector3.h"

#include <array>
#include <vector>

namespace dvision {

//! Information for a cluster of sample particles
struct Cluster {
  //! Number of samples
  int count;

  //! Total weight of samples in this cluster
  double weight;
  //! Average weight of samples in this cluster
  double meanWeight;

  //! Mean value for cluster statistics
  Pose mean;
  //! Covariance value for cluster statistics
  Matrix cov;

  //! Workspace for mean value
  double m[4];
  //! Workspace for covariance value
  double c[2][2];
};

//! Information for a set of sample particles
struct SampleSet {
  //! SampleSet destructor
  ~SampleSet() { delete kdtree; }

  //! Samples of particles
  std::vector<Particle> samples;
  //! A kdtree encoding the histogtam
  KdTree* kdtree = nullptr;

  // Clusters
  //! Count of clusters
  int cluster_count = 0;
  //! Maximum count of clusters
  int cluster_max_count;
  //! Set of clusters
  std::vector<Cluster> clusters;

  // Filter statistics
  //! Mean value for cluster statistics
  Pose mean;
  //! Covariance value for cluster statistics
  Matrix cov;
  //! Flag for whether or not sample set is converged
  bool converged = false;
};

/**
 * @brief class for Adaptive (or KLD-sampling) Monte Carlo localization
 */
class AMCL {
 public:
  //! AMCL constructor
  AMCL();
  //! Initialize AMCL
  void Init();

  /**
   * @brief Process localization
   * @param [in] z - detected measurement of land marks
   * @param [in] u - control assignment in motion module
   * @param [in] imu_yaw_degree - field angle
   * @param [in] crouching - whether the robot is crouching
   * @param [out] visionInfo - information of vision module to write
   * localization result
   */
  void Process(Measurement& z, Control& u, double imu_yaw_degree,
               dmsgs::VisionInfo& visionInfo, bool crouching);

  /**
   * @brief Get particles
   * @return set of particles
   */
  std::vector<Particle>& particles() { return sets_[current_set_].samples; }
  /**
   * @brief Get map
   * @return map
   */
  Map& map() { return map_; }
  /**
   * @brief Get estimated pose
   *
   * @return estimated pose
   */
  inline Pose GetEstimatedPose() { return pose_; }
  /**
   * @brief Determine whether or not localization is convergent
   *
   * @return whether or not localization is convergent
   */
  bool IsConverged() { return converged_; }

  /**
   * @brief Reset particles on touch line
   */
  void ResetParticlesTouchLine(const uint8_t& side);
  /**
   * @brief Reset particles in left half field
   */
  void ResetParticlesLeftTouch();
  /**
   * @brief Reset particles in right half field
   */
  void ResetParticlesRightTouch();
  /**
   * @brief Reset particles with given point
   *
   * @param [in] point - given particle point from message
   */
  void ResetParticlesPoint(geometry_msgs::Vector3 point);
  /**
   * @brief Reset particles with random pose
   */
  void ResetParticlesRandom();
  /**
   * @brief Apply gaussian distribution when robot falls down
   */
  void falldownGauss();
  /**
   * @brief Get field quality
   *
   * @return field quality
   */
  inline double GetQuality() { return quality_; }
  /**
   * @brief Get consistency
   *
   * @return consistency
   */
  inline double GetConsistency() { return consistency_; }
#ifdef USE_NEO_INFO
  /**
   * @brief Set yaw in amcl
   *
   */
  inline void SetYaw(double yaw) {
    yaw_ = yaw;
    return;
  }
#endif

 private:
  // TODO rename update to Update
  /**
   * @brief update AMCL estimation
   *
   * @param [in] z - detected measurement of land marks
   * @param [in] u - control assignment in motion module
   * @param [in] imu_yaw_degree - field angle
   */
  void Update(Measurement& z, Control& u, double imu_yaw_degree);
  /**
   * @brief Sample with motion model
   *
   * @param [in] u - control assignment in motion module
   */
  void SampleMotionModel(Control& u);
  /**
   * @brief Sample with Measurement Model
   *
   * @param [in] z - detected measurement of land marks
   */
  void MeasurementModel(Measurement& z);
  /**
   * @brief Resample with Measurement Model
   *
   * @param [in] z - detected measurement of land marks
   */
  void Resample(Measurement& z);
  //  void SensorReset(Measurement& z);
  /**
   * @brief Estimate location
   */
  void Estimate();
  /**
   * @brief Check whether or not sample set is convergent
   *
   * @return whether or not sample set is convergent
   */
  bool CheckConverged();
  /**
   * @brief Generate random pose
   *
   * @return a random pose
   */
  Pose RandomPose(const bool& use_yaw = false);
  /**
   * @brief Calculate statistics of cluster
   *
   * @param [in] set - sample set
   */
  void ClusterStats(SampleSet& set);
  /**
   * @brief Update consistency
   *
   * @param [in] z - detected measurement of land marks
   */
  void UpdateConsistency(Measurement& z);
  /**
   * @brief calculate score of detected mark position(goal posts or center)
   *
   * @param [in] detected_p - detected mark position(goal posts or center)
   *
   * @return the score of the detected mark position(goal posts or center)
   */
  double GetScore(cv::Point2f detected_p);

  //! Number of particles
  int num_particles_;
  //! Count of cycles
  int cycle_ = 0;

  //! The sample sets.
  //* We keep two sets and use [current_set] to identify the active set.
  SampleSet sets_[2];
  //! Index for current active set
  int current_set_ = 0;

  //! Map
  Map map_;
  //! Pose
  Pose pose_;

  // Parameters for AMCL
  //! Running averages (slow) of likelihood
  double w_slow_ = 0.0;
  //! Running averages (fast) of likelihood
  double w_fast_ = 0.0;

  //! Decay rates (slow) for running averages
  double alpha_slow_ = 0.0;
  //! Decay rates (fast) for running averages
  double alpha_fast_ = 0.0;

  // Parameters for laser probability model?
  double z_hit_ = 0.95;
  double z_rand_ = 0.95;
  double sigma_hit_ = 0.2;

  //! Cycle interval for resampling
  int resample_interval_ = 5;

  //! Distance threshold in each axis over which the pf is considered to not be
  //! converged
  double dist_threshold_ = 50;
  //! Flag for whether or not pf is converged
  bool converged_ = false;

  //! Field angle of robot
  double yaw_ = 0;

  // Gaissian noise
  //! Gaussian distribution for x axis
  Gaussian y_gauss_;
  //! Gaussian distribution for y axis
  Gaussian x_gauss_;
  //! Gaussian distribution for resampling
  Gaussian resample_gauss_;
  //! Uniform distribution
  Uniform uniform_;

  // Field quality
  double quality_;
  // Threshold to determin a particle is good or bad
  double good_dist_th_;
  double good_angle_th_;

  // Consistancy
  double consistency_;
  double consistency_step_cost_;
  double consistency_good_gain_;
  double consistency_bad_cost_;
  double consistency_th_;

  // Distance increase speed of force resample(cm per tick).
  int resample_dist_increase_;

  // Max distance between resample particle and current position
  int max_resample_dist_;

  // Cycle since last force resample
  int since_last_resample_;

  int estimate_interval_;

  // Whether the robot is stable enough(crouch) to get good marker position
  bool stable_crouching_ = false;
};

}  // namespace dvision

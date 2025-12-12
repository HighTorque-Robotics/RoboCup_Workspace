/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file amcl.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-22
 */

#include "dvision/amcl/amcl.hpp"

#include <dvision/timer.hpp>

#include "dancer_geometry/utils.hpp"
#include "dvision/parameters.hpp"

using namespace std;
using namespace dancer_geometry;

// FIXME(MWX): if all particles converged, but weight are small, then there's no
// difference between using 10 samples or 10000 samples

namespace dvision {

AMCL::AMCL()
    : current_set_(0),
      converged_(false),
      // don't change this!!!
      y_gauss_(0, .3),
      x_gauss_(0, .5),
      resample_gauss_(0, 5),
      uniform_(-1, 1),
      quality_(0),
      consistency_(1),
      consistency_step_cost_(0.0005),
      consistency_good_gain_(0.08),
      consistency_bad_cost_(0.05),
      since_last_resample_(0) {}

void AMCL::Init() {
  //! Initialize parameters
  // TODO check AMCL parameters
  // num_particles_ = parameters.amcl.num_particles;
  num_particles_ = 300;
  // resample_interval_ = 30 * 2;

  dist_threshold_ = parameters.amcl.dist_threshold;
  z_hit_ = parameters.amcl.z_hit;
  z_rand_ = parameters.amcl.z_rand;
  sigma_hit_ = parameters.amcl.sigma_hit;
  good_dist_th_ = parameters.amcl.good_dist_th;
  good_angle_th_ = parameters.amcl.good_angle_th;
  consistency_step_cost_ = parameters.amcl.c_step_cost;
  resample_interval_ = parameters.amcl.resample_interval;
  estimate_interval_ = parameters.amcl.estimate_interval;
  consistency_th_ = parameters.amcl.consistency_th;
  resample_dist_increase_ = parameters.amcl.resample_dist_increase;
  max_resample_dist_ = parameters.amcl.max_resample_dist;

  map_.Init();

  //! Initialize sample sets
  for (int i = 0; i < 2; ++i) {
    auto &set = sets_[i];

    set.samples.resize(num_particles_);
    set.kdtree = new KdTree(3 * num_particles_);
    set.cluster_max_count = num_particles_;
    set.clusters.resize((size_t)set.cluster_max_count);

    set.mean = Pose(0, 0, 0);
    set.cov = Matrix();
  }

  //! Initialize current set
  auto &set = sets_[current_set_];
  set.kdtree->Clear();
  for (auto &sample : set.samples) {
    sample.weight = 1.0 / set.samples.size();
    sample.pose = RandomPose();
    set.kdtree->InsertPose(sample.pose, sample.weight);
  }
  ClusterStats(set);
}

void AMCL::Process(Measurement &z, Control &u, double imu_yaw_degree,
                   dmsgs::VisionInfo &visionInfo, bool crouching) {
  Timer t;

  stable_crouching_ = crouching;
  //! Update estimation
  Update(z, u, imu_yaw_degree);

  //! Put result into visionInfo
  auto &particles = this->particles();
  int nGoodParticles = 0;

  if (parameters.amcl.debug_on_monitor)
    visionInfo.particles.resize(particles.size());

  for (uint32_t i = 0; i < particles.size(); ++i) {
    if (parameters.amcl.debug_on_monitor) {
      visionInfo.particles[i].pose.x = particles[i].pose.x();
      visionInfo.particles[i].pose.y = particles[i].pose.y();
      visionInfo.particles[i].pose.z = particles[i].pose.heading();
      visionInfo.particles[i].weight = particles[i].weight;
    }
    double dist =
        GetDistance(cv::Point2f(particles[i].pose.x(), particles[i].pose.y()),
                    cv::Point2f(pose_.x(), pose_.y()));
    // double d_angle_rad = Degree2Radian(particles[i].pose.heading() -
    // pose_.heading());
    double d_angle = std::fabs(particles[i].pose.heading() - pose_.heading());
    // count good particles
    if (dist < good_dist_th_ && d_angle < good_angle_th_) nGoodParticles++;
  }

  visionInfo.loc_ok = converged_;
  visionInfo.robot_pos.x = pose_.x();
  visionInfo.robot_pos.y = pose_.y();
  visionInfo.robot_pos.z = pose_.heading();
  quality_ = nGoodParticles / (double)particles.size();
  UpdateConsistency(z);
  ROS_DEBUG("AMCL used: %lf ms", t.elapsedMsec());
}

void AMCL::Update(Measurement &z, Control &u, double imu_yaw_degree) {
  //! Update predictions
  yaw_ = imu_yaw_degree;
  //! Resample

  SampleMotionModel(u);
  if ((++cycle_ > resample_interval_ && consistency_ < consistency_th_) ||
      !z.center_point.empty() || !z.goal_center.empty() ||
      !z.corner_points.empty()) {
    Resample(z);
    cycle_ = 0;
  }
  MeasurementModel(z);

  //! Estimate
  if (!(cycle_ % estimate_interval_)) {
    Estimate();
    CheckConverged();
  }
}

void AMCL::SampleMotionModel(Control &u) {
  double dx = u.dx;
  double dy = u.dy;
  //std::cout << "dx = " << dx << ",dy = " << dy << std::endl;
  double w = dconstant::geometry::fieldLength / 2 +
             dconstant::geometry::borderStripWidth;
  double h = dconstant::geometry::fieldWidth / 2 +
             dconstant::geometry::borderStripWidth;

  for (auto &particle : this->particles()) {
    Pose &p = particle.pose;
    double ddx = dx + x_gauss_.sample();
    double ddy = dy + y_gauss_.sample();

    double heading = Degree2Radian(p.heading());
    if (parameters.simulation) {
      p.setX(p.x() + ddx * std::cos(heading) - ddy * std::sin(heading));
      p.setY(p.y() + ddx * std::sin(heading) + ddy * std::cos(heading));
    } else {
      p.setX(p.x() + ddx);
      p.setY(p.y() + ddy);
    }
    p.setHeading(yaw_);

    p.setX(std::min(p.x(), w));
    p.setX(std::max(p.x(), -w));
    p.setY(std::min(p.y(), h));
    p.setY(std::max(p.y(), -h));
  }
}

void AMCL::MeasurementModel(Measurement &z) {
  double total_weight = 0;
  auto &set = sets_[current_set_];

  float max_dist_error = 500.0, dist_error, dist;
  for (auto &sample : set.samples) {
    double prob = sample.weight;
    cv::Point3d global_pos(sample.pose.x(), sample.pose.y(),
                           sample.pose.heading());
    // Ignore close features due to location error when walking
    if (z.field_points.size() > 1 ||
        (z.field_points.size() == 1 &&
         (z.field_points[0].type == dmsgs::FieldFeature::T_INTXN ||
          z.field_points[0].type == dmsgs::FieldFeature::PENALTY_MARK)))
      // field features
      for (auto &field_pt : z.field_points) {
        dist = dancer_geometry::GetDistance(field_pt.pred);
        if (dist > parameters.amcl.max_dist)
          continue;
        cv::Point2f pt_global =
            getOnGlobalCoordinate(global_pos, field_pt.pred);
        dist_error = max_dist_error;
        for (auto &ref : field_pt.ref)
          dist_error =
              min(dist_error, dancer_geometry::GetDistance(pt_global, ref));

        // Ignore distance larger than max_dist_error
        if (max_dist_error - dist_error < 1) continue;
        // Norm distance value to [0, 10].
        // Lower distance for higher score(prob) in gaussian distribution.
        dist_error /= (max_dist_error / 10);
        double pz = normal_pdf(double(dist_error), 0.0, sigma_hit_);
        prob += pz * field_pt.weight;
      }

    // Center
    for (auto &center : z.center_point) {
      dist = dancer_geometry::GetDistance(center.pred);
      if (dist > parameters.amcl.max_dist)
        continue;
      cv::Point2d gCenter = getOnGlobalCoordinate(global_pos, center.pred);
      // divide by 5.0, like in occ dist_error map
      dist_error = sqrt(gCenter.x * gCenter.x + gCenter.y * gCenter.y);
      dist_error = std::min(dist_error, max_dist_error);
      // Ignore distance larger than max_dist_error
      if (max_dist_error - dist_error < 1) continue;
      dist_error /= (max_dist_error / 10);
      double pz = normal_pdf(double(dist_error), 0.0, sigma_hit_);
      prob += pz * center.weight;
    }

    // Goal center
    for (auto &goal : z.goal_center) {
      dist = dancer_geometry::GetDistance(goal.pred);
      cv::Point2d gCenter = getOnGlobalCoordinate(global_pos, goal.pred);
      dist_error = max_dist_error;
      for (auto &ref : goal.ref) {
        auto dx = ref.x - gCenter.x;
        auto dy = gCenter.y;
        dist_error = std::min(sqrt(dx * dx + dy * dy), double(dist_error));
      }
      // Ignore distance larger than max_dist_error
      if (max_dist_error - dist_error < 1) continue;
      dist_error /= (max_dist_error / 10);
      double pz = normal_pdf(double(dist_error), 0.0, sigma_hit_);
      prob += pz * goal.weight;
    }

    // Update weight slower
    sample.weight = prob / 10.;
    total_weight += sample.weight;
  }

  if (total_weight > 0) {
    // Normalize weights
    for (auto &sample : set.samples) {
      sample.weight /= total_weight;
    }

    double w_avg = total_weight / set.samples.size();
    if (w_slow_ == 0.0) {
      w_slow_ = w_avg;
    } else {
      w_slow_ += alpha_slow_ * (w_avg - w_slow_);
    }

    if (w_fast_ == 0.0) {
      w_fast_ = w_avg;
    } else {
      w_fast_ += alpha_fast_ * (w_avg - w_fast_);
    }
  } else {
    for (auto &sample : set.samples) sample.weight = 1.0 / set.samples.size();
  }
}

void AMCL::Resample(Measurement &z) {
  auto &set_a = sets_[current_set_];
  auto &set_b = sets_[(current_set_ + 1) % 2];
  std::random_shuffle(set_b.samples.begin(), set_b.samples.end());
  double b_nums = set_b.samples.size();
  double max_weight = 0;

  double c[set_a.samples.size() + 1];
  c[0] = 0.0;
  for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
    c[i + 1] = c[i] + set_a.samples[i].weight;
    if (set_a.samples[i].weight > max_weight)
      max_weight = set_a.samples[i].weight;
  }

  since_last_resample_++;
  size_t cnt = 0;
  if (!z.center_point.empty() && stable_crouching_) {
    // calc pose with max likelihood observing the center
    // and put some samples there
    size_t reset_for_center_size = set_b.samples.size() / 3;
    auto &center = z.center_point[0];
    double x = center.pred.x;
    double y = center.pred.y;
    double dist = sqrt(x * x + y * y);
    double r = pose_.headingR();

    auto dx = x * cos(r) - y * sin(r);
    auto dy = x * sin(r) + y * cos(r);

    auto xx = 0.0 - dx;
    auto yy = 0.0 - dy;
    int resample_dist = std::min(since_last_resample_ * resample_dist_increase_,
                                 max_resample_dist_);
    if (dist < parameters.amcl.max_dist &&
        GetDistance(cv::Point2f(xx, yy), cv::Point2f(pose_.x(), pose_.y())) <
            resample_dist) {
      since_last_resample_ = 0;
      for (; cnt < reset_for_center_size; ++cnt) {
        set_b.samples[cnt].pose =
            Pose(xx + resample_gauss_.sample(), yy + resample_gauss_.sample(),
                 pose_.heading());
        set_b.samples[cnt].weight = 1.0 / b_nums;
      }
      // ROS_INFO("Resample when see circle!");
    }
  }
  if (!z.goal_center.empty() && stable_crouching_) {
    // calc pose with maxlikelihood observing the goal post
    // and push some samples there
    size_t reset_size = set_b.samples.size() / 3;
    auto goalX = dconstant::geometry::field_length_half;
    auto goalY = 0;

    auto &goalCenter = z.goal_center[0].pred;

    double x = goalCenter.x;
    double y = goalCenter.y;
    double dist = sqrt(x * x + y * y);
    // Ignore goal that is too far when not crouching
    if (dist < parameters.amcl.max_dist) {
      double r = pose_.headingR();
      auto dx = x * cos(r) - y * sin(r);
      auto dy = x * sin(r) + y * cos(r);

      double xx;
      bool leftField;
      if (dx > 0) {
        xx = goalX - dx;
        leftField = false;
      } else {
        xx = -goalX - dx;
        leftField = true;
      }

      for (size_t i = 0; i < reset_size; ++i) {
        double newY;
        if (leftField)
          newY = -goalY - dy;
        else
          newY = goalY - dy;

        int resample_dist = std::min(
            since_last_resample_ * resample_dist_increase_, max_resample_dist_);
        if (GetDistance(cv::Point2f(xx, newY),
                        cv::Point2f(pose_.x(), pose_.y())) < resample_dist) {
          since_last_resample_ = 0;
          set_b.samples[cnt].pose =
              Pose(xx + resample_gauss_.sample(),
                   newY + resample_gauss_.sample(), pose_.heading());
          set_b.samples[cnt].weight = 1.0 / b_nums;
          ++cnt;
        }
      }

      // ROS_INFO("Goal Resample");
    }
  }

  Pose high_p = pose_;
  double x = high_p.x();
  double y = high_p.y();

  // P number of the new particles are drawn around the old particles with the
  // highest particle weight.
  size_t reset_for_heavy_small = max_weight * set_b.samples.size();

  for (; cnt < reset_for_heavy_small; ++cnt) {
    set_b.samples[cnt].pose =
        Pose(x + resample_gauss_.sample(), y + resample_gauss_.sample(),
             high_p.heading());
    set_b.samples[cnt].weight = 1.0 / b_nums;
  }
  for (; cnt < set_b.samples.size(); ++cnt) {
    auto &sample_b = set_b.samples[cnt];

    sample_b.pose = Pose(x + 5 * resample_gauss_.sample(),
                         y + 5 * resample_gauss_.sample(), high_p.heading());
    sample_b.weight = 1.0 / b_nums;
  }
  // ROS_INFO("Custom Resample");

  // } else {
  //   for (; cnt < set_b.samples.size(); ++cnt) {
  //     auto &sample_b = set_b.samples[cnt];

  //     set_b.samples[cnt].pose =
  //         Pose(x + 3 * resample_gauss_.sample(),
  //              y + 3 * resample_gauss_.sample(), high_p.heading());
  //     sample_b.weight = 1.0 / b_nums;
  //   }
  // }

  double w_diff = 1.0 - w_fast_ / w_slow_;
  w_diff = max(0.0, w_diff);

  for (unsigned int cnt = 0; cnt < set_b.samples.size(); ++cnt) {
    auto &sample_b = set_b.samples[cnt];

    if (drand48() < w_diff) {
      for (uint32_t i = 0; i < set_a.samples.size(); ++i) {
        sample_b = set_a.samples[i];
        sample_b.pose.setX(sample_b.pose.x() + resample_gauss_.sample());
        sample_b.pose.setY(sample_b.pose.y() + resample_gauss_.sample());
      }
    }
    sample_b.weight = 1.0 / b_nums;
  }

  current_set_ = (current_set_ + 1) % 2;

  CheckConverged();
}

void AMCL::Estimate() {
  //  double x_sum = 0, y_sum = 0, z_sum = 0;
  //  for(auto& sample : this->particles()) {
  //      x_sum += sample.pose.x();
  //      y_sum += sample.pose.y();
  //      z_sum += sample.pose.heading();
  //  }
  //  auto num_particles = this->particles().size();
  //  pose_.setX(x_sum / num_particles);
  //  pose_.setY(y_sum / num_particles);
  //  pose_.setHeading(z_sum / num_particles);

  // Create the kd tree for adaptive sampling
  auto &set_b = sets_[current_set_];
  set_b.kdtree->Clear();

  for (auto &sample : set_b.samples) {
    set_b.kdtree->InsertPose(sample.pose, sample.weight);
  }

  // Re-compute cluster statistics
  ClusterStats(set_b);

  auto &set = sets_[current_set_];
  double max_weight = 0.0;
  for (auto &cluster : set.clusters) {
    if (cluster.weight > max_weight) {
      max_weight = cluster.weight;
      pose_ = cluster.mean;
    }
  }
}

bool AMCL::CheckConverged() {
  auto &set = sets_[current_set_];

  double mean_x = 0, mean_y = 0;
  for (auto &sample : set.samples) {
    mean_x += sample.pose.x();
    mean_y += sample.pose.y();
  }
  mean_x /= set.samples.size();
  mean_y /= set.samples.size();

  for (auto &sample : set.samples) {
    if (fabs(sample.pose.x() - mean_x) > this->dist_threshold_ ||
        fabs(sample.pose.y() - mean_y) > this->dist_threshold_) {
      set.converged = false;
      converged_ = false;
      return false;
    }
  }
  converged_ = true;
  set.converged = true;
  return true;
}

void AMCL::ClusterStats(SampleSet &set) {
  //! Initialize workspace
  double m[4], c[2][2];
  set.kdtree->Cluster();

  //! Initialize cluster stats
  set.cluster_count = 0;

  for (int i = 0; i < set.cluster_max_count; ++i) {
    Cluster &cluster = set.clusters[i];
    cluster.count = 0;
    cluster.weight = 0;
    cluster.mean = Pose();
    cluster.cov = Matrix();

    for (int j = 0; j < 4; ++j) cluster.m[j] = 0.0;

    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) cluster.c[j][k] = 0.0;
  }

  //! Initialize overall filter stats
  size_t count = 0;
  double weight = 0.0;
  set.mean = Pose();
  set.cov = Matrix();

  for (int j = 0; j < 4; ++j) m[j] = 0.0;

  for (int j = 0; j < 2; ++j)
    for (int k = 0; k < 2; ++k) c[j][k] = 0.0;

  //! Compute cluster stats
  for (auto &sample : set.samples) {
    // Get the cluster label for this sample
    int cidx = set.kdtree->GetCluster(sample.pose);
    // assert(cidx >= 0);

    if (cidx >= set.cluster_max_count) continue;
    if (cidx + 1 > set.cluster_count) set.cluster_count = cidx + 1;

    auto &cluster = set.clusters[cidx];

    cluster.count += 1;
    cluster.weight += sample.weight;

    count += 1;
    weight += sample.weight;

    // Compute mean
    cluster.m[0] += sample.weight * sample.pose.x();
    cluster.m[1] += sample.weight * sample.pose.y();
    cluster.m[2] += sample.weight * std::cos(sample.pose.headingR());
    cluster.m[3] += sample.weight * std::sin(sample.pose.headingR());

    m[0] += sample.weight * sample.pose.x();
    m[1] += sample.weight * sample.pose.y();
    m[2] += sample.weight * std::cos(sample.pose.headingR());
    m[3] += sample.weight * std::sin(sample.pose.headingR());

    // Compute covariance in linear components
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) {
        cluster.c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
        c[j][k] += sample.weight * sample.pose[j] * sample.pose[k];
      }
  }

  //! Normalize
  for (int i = 0; i < set.cluster_count; ++i) {
    auto &cluster = set.clusters[i];
    cluster.mean.setX(cluster.m[0] / cluster.weight);
    cluster.mean.setY(cluster.m[1] / cluster.weight);
    cluster.mean.setHeadingR(atan2(cluster.m[3], cluster.m[2]));

    cluster.cov = Matrix();

    // Covariance in linear components
    for (int j = 0; j < 2; ++j)
      for (int k = 0; k < 2; ++k) {
        cluster.cov.m[j][k] = cluster.c[j][k] / cluster.weight -
                              cluster.mean[j] * cluster.mean[k];
      }
    // Covariance in angular components;
    cluster.cov.m[2][2] =
        -2 *
        log(sqrt(cluster.m[2] * cluster.m[2] + cluster.m[3] * cluster.m[3]));
    // Cluster mean weight
    cluster.meanWeight = cluster.weight / cluster.count;
  }

  //! Compute overall filter stats
  set.mean[0] = m[0] / weight;
  set.mean[1] = m[1] / weight;
  set.mean.setHeadingR(atan2(m[3], m[2]));

  //! Covariance in linear components
  for (int j = 0; j < 2; ++j) {
    for (int k = 0; k < 2; ++k) {
      set.cov.m[j][k] = c[j][k] / weight - set.mean[j] * set.mean[k];
    }
  }

  set.cov.m[2][2] = -2 * log(sqrt(m[2] * m[2] + m[3] * m[3]));
}

Pose AMCL::RandomPose(const bool &use_yaw) {
  double w = dconstant::geometry::fieldLength / 2;
  double h = dconstant::geometry::fieldWidth / 2;

  Pose p;
  p.setX(uniform_.sample() * w);
  p.setY(uniform_.sample() * h);
  if (use_yaw) {
    p.setHeading(yaw_);
  } else {
    p.setHeading(uniform_.sample() * 180);
  }
  return p;
}

void AMCL::ResetParticlesTouchLine(const uint8_t &side) {
  if (side == dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_BOTH) {
    ResetParticlesLeftTouch();
  } else if (side ==
             dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_BOTH) {
    ResetParticlesRightTouch();
  } else if (side ==
             dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_TOP) {
    auto xUniform = Uniform(-450, 0);
    auto &cur_set = sets_[current_set_];
    for (auto &sample : cur_set.samples) {
      sample.pose = Pose(xUniform.sample(), 300, yaw_);
    }
    for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
      sample.pose = Pose(xUniform.sample(), 300, yaw_);
    }
  } else if (side ==
             dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_LEFT_BOTTOM) {
    auto xUniform = Uniform(-450, 0);
    auto &cur_set = sets_[current_set_];
    for (auto &sample : cur_set.samples) {
      sample.pose = Pose(xUniform.sample(), -300, yaw_);
    }
    for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
      sample.pose = Pose(xUniform.sample(), -300, yaw_);
    }
  } else if (side ==
             dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_TOP) {
    auto xUniform = Uniform(450, 0);
    auto &cur_set = sets_[current_set_];
    for (auto &sample : cur_set.samples) {
      sample.pose = Pose(xUniform.sample(), 300, yaw_);
    }
    for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
      sample.pose = Pose(xUniform.sample(), 300, yaw_);
    }
  } else if (side ==
             dmsgs::ResetParticleTouchLineRequest::TOUCH_LINE_RIGHT_BOTTOM) {
    auto xUniform = Uniform(450, 0);
    auto &cur_set = sets_[current_set_];
    for (auto &sample : cur_set.samples) {
      sample.pose = Pose(xUniform.sample(), -300, yaw_);
    }
    for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
      sample.pose = Pose(xUniform.sample(), -300, yaw_);
    }
  }
}

void AMCL::ResetParticlesLeftTouch() {
  // from (-450, 300) to (0, 300)
  // from (-450, -300) to (0, -300)
  auto xUniform = Uniform(-450, 0);
  auto &cur_set = sets_[current_set_];
  size_t i;
  for (i = 0; i < cur_set.samples.size() / 2; ++i) {
    cur_set.samples[i].pose = Pose(xUniform.sample(), 300, yaw_);
  }
  for (; i < cur_set.samples.size(); ++i) {
    cur_set.samples[i].pose = Pose(xUniform.sample(), -300, yaw_);
  }
  for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
    sample.pose = Pose(xUniform.sample(), 300, yaw_);
  }
  for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
    sample.pose = Pose(xUniform.sample(), -300, yaw_);
  }
}

void AMCL::ResetParticlesRightTouch() {
  // from (450, 300) to (0, 300)
  // from (450, -300) to (0, -300)
  auto xUniform = Uniform(450, 0);
  auto &cur_set = sets_[current_set_];
  size_t i;
  for (i = 0; i < cur_set.samples.size() / 2; ++i) {
    cur_set.samples[i].pose = Pose(xUniform.sample(), 300, yaw_);
  }
  for (; i < cur_set.samples.size(); ++i) {
    cur_set.samples[i].pose = Pose(xUniform.sample(), -300, yaw_);
  }
  for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
    sample.pose = Pose(xUniform.sample(), 300, yaw_);
  }
  for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
    sample.pose = Pose(xUniform.sample(), -300, yaw_);
  }
}

void AMCL::ResetParticlesPoint(geometry_msgs::Vector3 point) {
  Gaussian fuck(0, 3);
  auto &cur_set = sets_[current_set_];
  for (auto &sample : cur_set.samples) {
    sample.pose =
        Pose(point.x + fuck.sample(), point.y + fuck.sample(), point.z);
  }
  for (auto &sample : sets_[(current_set_ + 1) % 2].samples) {
    sample.pose =
        Pose(point.x + fuck.sample(), point.y + fuck.sample(), point.z);
  }
}

void AMCL::ResetParticlesRandom() {
  Gaussian fuck(0, 3);
  auto &cur_set = sets_[current_set_];
  for (auto &sample : cur_set.samples) {
    sample.pose = RandomPose(true);
  }
}

void AMCL::falldownGauss() {
  Gaussian g(0, 10);
  for (auto &sample : sets_[current_set_].samples) {
    auto x = sample.pose.x();
    auto y = sample.pose.y();

    sample.pose.setX(x + g.sample());
    sample.pose.setY(y + g.sample());
  }
}

void AMCL::UpdateConsistency(Measurement &z) {
  // calculate step_delta_score
  double step_delta_score = -consistency_step_cost_;
  for (auto goal : z.goal_center) {
    double score = GetScore(goal.pred);
    // TODO(dai-z) set min_score for every goal measurement or a param in yml
    double min_score = 0.5;
    if (score > min_score)
      step_delta_score += consistency_good_gain_;
    else
      step_delta_score -= consistency_bad_cost_;
  }
  for (auto center : z.center_point) {
    double score = GetScore(center.pred);
    // TODO(dai-z) set min_score a param in yml
    double min_score = 0.5;
    if (score > min_score)
      step_delta_score += consistency_good_gain_;
    else
      step_delta_score -= consistency_bad_cost_;
  }
  // update consistency score
  consistency_ += step_delta_score;
  consistency_ = std::min(1.0, std::max(0.0, consistency_));
}

double AMCL::GetScore(cv::Point2f detected_p) {
  double w = dconstant::geometry::goalWidth / 2;
  double l = dconstant::geometry::fieldLength / 2;
  std::vector<cv::Point2f> global_field_mark;
  // Goal posts
  global_field_mark.push_back(cv::Point2f(l, w));
  global_field_mark.push_back(cv::Point2f(l, -w));
  global_field_mark.push_back(cv::Point2f(-l, w));
  global_field_mark.push_back(cv::Point2f(-l, -w));
  // Center circle
  global_field_mark.push_back(cv::Point2f(0, 0));

  double best_score = 0;
  for (auto m : global_field_mark) {
    // Calculated by actual goal position and robot position
    cv::Point2f expectedPos = m - cv::Point2f(pose_.x(), pose_.y());
    // In robot coordinate
    cv::Point2f expectedPos_robot =
        RotateCoordinateAxis(-pose_.heading(), expectedPos);
    double dist_diff = GetDistance(expectedPos_robot, detected_p);
    // double goal_direction = g.x > 0 ? 0 : 180;
    // double angle_diff = std::fabs(goal_direction - pose_.heading());

    double dist_score, dist_error_tolerance = 0.0;
    // double angle_score, angle_error_tolerance = 0.0;
    // TODO(dai-z) set error_tolerance > 0
    dist_score = 1 - (dist_diff - dist_error_tolerance) /
                         (good_dist_th_ - dist_error_tolerance);
    // angle_score = 1 - (angle_diff - angle_error_tolerance) / (good_angle_th_
    // - angle_error_tolerance);
    // angle_score = 0;
    // best_score = std::min(1.0, std::max(best_score, std::max(dist_score,
    // angle_score)));
    // Use dist_score only
    best_score = std::min(1.0, std::max(best_score, dist_score));
  }
  // TODO(dai-z) take error into consideration, e.g. best = best*(1-error)+error
  // TODO(dai-z) take weight into consideration, e.g. best = pow(best,
  // 1+(weight-1)*weightRatio)
  return best_score;
}
}  // namespace dvision

#include "dvision/corner_detector.hpp"

#include "dconfig/dconstant.hpp"
#include "dvision/amcl/types.hpp"
#include "dvision/parameters.hpp"

namespace dvision {
CornerDetector::CornerDetector() {}

CornerDetector::~CornerDetector() {}

bool CornerDetector::Init() {
  if (!parameters.corner.enable) {
    return false;
  }
  min_x_ = parameters.camera.width * 0.05;
  min_y_ = parameters.camera.height * 0.05;
  max_x_ = parameters.camera.width - min_x_;
  max_y_ = parameters.camera.height - min_y_;

  single_features_ = parameters.field_features.all_points;

  ROS_DEBUG("CornerDetector Init");
  return true;
}

void CornerDetector::Detect(FieldFeatures& field_features,
                            Projection& projection, VisionInfo& vision_info,
                            cv::Mat& gui_img, Measurement& mark) {
  l_center_.clear();
  t_center_.clear();
  x_center_.clear();
  penalty_center_.clear();

  // Detect intersections with simple features(L, T, X intersections)
  yoloDetect(field_features, vision_info, projection);
  classifyIntx(mark, projection);

  // ROS_DEBUG("Corner detect used: %lf ms", t.elapsedMsec());
}

// save the rotate-toward-heading position of t x l penalty marks
void CornerDetector::yoloDetect(FieldFeatures& field_features,
                                VisionInfo& vision_info,
                                Projection& projection) {
  dmsgs::FieldFeature msg_pt;
  cv::Point img_pt;
  for (const auto& t_box : field_features.t) {
    if (boxInBoundary(t_box)) continue;
    img_pt.x = (t_box.left_top.x + t_box.right_bottom.x) / 2;
    img_pt.y = (t_box.left_top.y + t_box.right_bottom.y) / 2;
    msg_pt.feature = dmsgs::FieldFeature::T_INTXN;
    t_center_.push_back(cv::Point2f(0, 0));
    projection.getOnRealCoordinate(cv::Point(img_pt.x, img_pt.y),
                                   t_center_.back());
    msg_pt.x = t_center_.back().x;
    msg_pt.y = t_center_.back().y;
    vision_info.features_field.push_back(msg_pt);
  }
  for (const auto& x_box : field_features.x) {
    if (boxInBoundary(x_box)) continue;
    img_pt.x = (x_box.left_top.x + x_box.right_bottom.x) / 2;
    img_pt.y = (x_box.left_top.y + x_box.right_bottom.y) / 2;
    msg_pt.feature = dmsgs::FieldFeature::X_INTXN;
    x_center_.push_back(cv::Point2f(0, 0));
    projection.getOnRealCoordinate(cv::Point(img_pt.x, img_pt.y),
                                   x_center_.back());
    msg_pt.x = x_center_.back().x;
    msg_pt.y = x_center_.back().y;
    vision_info.features_field.push_back(msg_pt);
  }
  for (const auto& l_box : field_features.l) {
    if (boxInBoundary(l_box)) continue;
    img_pt.x = (l_box.left_top.x + l_box.right_bottom.x) / 2;
    img_pt.y = (l_box.left_top.y + l_box.right_bottom.y) / 2;
    msg_pt.feature = dmsgs::FieldFeature::L_INTXN;
    l_center_.push_back(cv::Point2f(0, 0));
    projection.getOnRealCoordinate(cv::Point(img_pt.x, img_pt.y),
                                   l_center_.back());
    msg_pt.x = l_center_.back().x;
    msg_pt.y = l_center_.back().y;
    vision_info.features_field.push_back(msg_pt);
  }
  for (const auto& p_box : field_features.penalty) {
    if (boxInBoundary(p_box)) continue;
    img_pt.x = (p_box.left_top.x + p_box.right_bottom.x) / 2;
    img_pt.y = (p_box.left_top.y + p_box.right_bottom.y) / 2;
    msg_pt.feature = dmsgs::FieldFeature::PENALTY_MARK;
    penalty_center_.push_back(cv::Point2f(0, 0));
    projection.getOnRealCoordinate(cv::Point(img_pt.x, img_pt.y),
                                   penalty_center_.back());
    msg_pt.x = penalty_center_.back().x;
    msg_pt.y = penalty_center_.back().y;
    // std::cout<<msg_pt.x<<"---"<<msg_pt.y<<std::endl;
    vision_info.features_field.push_back(msg_pt);
  }
}

void CornerDetector::classifyIntx(Measurement& mark, Projection& projection) {
  // Common T intx
  for (auto& t_intx : t_center_) {
    mark.field_points.emplace_back(Measurement::LandMark());
    mark.field_points.back().pred.x = t_intx.x;
    mark.field_points.back().pred.y = t_intx.y;
    mark.field_points.back().ref = single_features_[0];
    mark.field_points.back().type = dmsgs::FieldFeature::T_INTXN;
    mark.field_points.back().weight = Measurement::getWeight(
        10, dancer_geometry::GetDistance(t_intx), parameters.amcl.trust_dist,
        parameters.amcl.max_dist);
    // for (int i = -1; i < 1; i += 2)
    //   for (int j = -1; j < 1; j += 2)
    //     mark.field_points.back().ref.emplace_back(
    //         i * dconstant::geometry::field_length_half,
    //         j * dconstant::geometry::goal_area_width_half);
    // for (int i = -1; i < 1; i += 2)
    //   mark.field_points.back().ref.emplace_back(
    //       0, i * dconstant::geometry::field_width_half);
  }
  // Common X intx
  for (auto& x_intx : x_center_) {
    mark.field_points.emplace_back(Measurement::LandMark());
    mark.field_points.back().pred.x = x_intx.x;
    mark.field_points.back().pred.y = x_intx.y;
    // mark.field_points.back().ref.emplace_back(
    //     0, dconstant::geometry::center_circle_radius);
    // mark.field_points.back().ref.emplace_back(
    //     0, -dconstant::geometry::center_circle_radius);
    mark.field_points.back().ref = single_features_[1];
    mark.field_points.back().type = dmsgs::FieldFeature::X_INTXN;
    mark.field_points.back().weight = Measurement::getWeight(
        20, dancer_geometry::GetDistance(x_intx), parameters.amcl.trust_dist,
        parameters.amcl.max_dist);
  }

  int goal_area_pos_x = dconstant::geometry::field_length_half -
                        dconstant::geometry::goalAreaLength;
  // Common L intx
  for (auto& l_intx : l_center_) {
    mark.field_points.emplace_back(Measurement::LandMark());
    mark.field_points.back().pred.x = l_intx.x;
    mark.field_points.back().pred.y = l_intx.y;
    mark.field_points.back().ref = single_features_[2];
    mark.field_points.back().type = dmsgs::FieldFeature::L_INTXN;
    mark.field_points.back().weight = Measurement::getWeight(
        10, dancer_geometry::GetDistance(l_intx), parameters.amcl.trust_dist,
        parameters.amcl.max_dist);
    // for (int i = -1; i < 1; i += 2)
    //   for (int j = -1; j < 1; j += 2)
    //     mark.field_points.back().ref.emplace_back(
    //         i * goal_area_pos_x, j *
    //         dconstant::geometry::goal_area_width_half);
    // for (int i = -1; i < 1; i += 2)
    //   for (int j = -1; j < 1; j += 2)
    //     mark.field_points.back().ref.emplace_back(
    //         i * dconstant::geometry::field_length_half,
    //         j * dconstant::geometry::field_width_half);
  }
  // Penalty mark
  for (auto& p : penalty_center_) {
    mark.field_points.emplace_back(Measurement::LandMark());
    mark.field_points.back().pred.x = p.x;
    mark.field_points.back().pred.y = p.y;
    mark.field_points.back().ref = single_features_[3];
    mark.field_points.back().type = dmsgs::FieldFeature::PENALTY_MARK;
    mark.field_points.back().weight = Measurement::getWeight(
        25, dancer_geometry::GetDistance(p), parameters.amcl.trust_dist,
        parameters.amcl.max_dist);
  }

  // float std_dist, angle;
  // cv::Point2f ref_pt;
  // // L + L
  // std_dist = dconstant::geometry::field_width_half -
  //            dconstant::geometry::goal_area_width_half;
  // ref_pt.x = (dconstant::geometry::goal_area_width_half +
  //             dconstant::geometry::field_width_half) /
  //            2.;
  // ref_pt.y = dconstant::geometry::field_length_half -
  //            dconstant::geometry::goal_area_width_half / 2.;
  // for (size_t i = 0; i < l_center_.size(); i++) {
  //   for (size_t j = 0; j < l_center_.size(); j++) {
  //     if (i == j) continue;
  //     dancer_geometry::LineSegment line(l_center_[i], l_center_[j]);
  //     if (line.checkLength(std_dist)) {
  //       mark.field_points.emplace_back(Measurement::LandMark());
  //       mark.field_points.back().pred = line.GetMiddle();
  //       mark.field_points.back().weight = Measurement::getWeight(
  //           20, dancer_geometry::GetDistance(line.GetMiddle()),
  //           parameters.amcl.trust_dist, parameters.amcl.max_dist);
  //       for (int ii = -1; ii < 1; ii += 2)
  //         for (int jj = 1; jj < 1; jj += 2)
  //           mark.field_points.back().ref.emplace_back(ii * ref_pt.x,
  //                                                     jj * ref_pt.y);
  //     }
  //   }
  // }

  // // L + T
  // int idx_0 = -1, idx_90 = -1;
  // std::vector<dancer_geometry::LineSegment> lt_on_robot;
  // dancer_geometry::LineSegment lt_0(
  //     cv::Point2f(dconstant::geometry::field_length_half,
  //                 dconstant::geometry::goal_area_width_half),
  //     cv::Point2f(goal_area_pos_x,
  //     dconstant::geometry::goal_area_width_half)),
  //     lt_90(cv::Point2f(dconstant::geometry::field_length_half,
  //                       dconstant::geometry::goal_area_width_half),
  //           cv::Point2f(dconstant::geometry::field_length_half,
  //                       dconstant::geometry::goal_area_width_half));
  // for (auto& l : l_center_)
  //   for (auto& t : t_center_)
  //     lt_on_robot.emplace_back(dancer_geometry::LineSegment(l, t));
  // std::vector<dancer_geometry::LineSegment> rotated_lt =
  //     projection.RotateTowardHeading(lt_on_robot);

  // for (size_t i = 0; i < rotated_lt.size(); i++) {
  //   angle = rotated_lt[i].GetAbsMinAngleDegree(lt_0);
  //   if (angle < 10) {
  //     if (rotated_lt[i].checkLength(lt_0.GetLength())) {
  //       idx_0 = i;
  //       mark.field_points.emplace_back(Measurement::LandMark());
  //       mark.field_points.back().pred = lt_on_robot[i].GetMiddle();
  //       mark.field_points.back().weight = Measurement::getWeight(
  //           20, dancer_geometry::GetDistance(lt_on_robot[i].GetMiddle()),
  //           parameters.amcl.trust_dist, parameters.amcl.max_dist);
  //       ref_pt = lt_0.GetMiddle();
  //       for (int ii = -1; ii < 1; ii += 2)
  //         for (int jj = -1; jj < 1; jj += 2)
  //           mark.field_points.back().ref.emplace_back(ii * ref_pt.x,
  //                                                     jj * ref_pt.y);
  //     }
  //   } else if (angle > 80) {
  //     if (rotated_lt[i].checkLength(lt_90.GetLength())) {
  //       idx_90 = i;
  //       mark.field_points.emplace_back(Measurement::LandMark());
  //       mark.field_points.back().pred = lt_on_robot[i].GetMiddle();
  //       mark.field_points.back().weight = Measurement::getWeight(
  //           20, dancer_geometry::GetDistance(lt_on_robot[i].GetMiddle()),
  //           parameters.amcl.trust_dist, parameters.amcl.max_dist);
  //       ref_pt = lt_90.GetMiddle();
  //       for (int ii = -1; ii < 1; ii += 2)
  //         for (int jj = -1; jj < 1; jj += 2)
  //           mark.field_points.back().ref.emplace_back(ii * ref_pt.x,
  //                                                     jj * ref_pt.y);
  //     }
  //   }
  // }

  // // L + T + L
  // if (idx_0 < 0 || idx_90 < 0) return;
  // ref_pt.x =
  //     (dconstant::geometry::field_length_half * 2 + goal_area_pos_x) / 3.;
  // ref_pt.y = (dconstant::geometry::goal_area_width_half * 2 +
  //             dconstant::geometry::field_width_half) /
  //            3.;
  // mark.field_points.emplace_back(Measurement::LandMark());
  // mark.field_points.back().pred.x =
  //     (lt_on_robot[idx_0].P1.x + lt_on_robot[idx_90].P1.x +
  //      lt_on_robot[idx_0].P2.x) /
  //     3;
  // mark.field_points.back().pred.y =
  //     (lt_on_robot[idx_0].P1.y + lt_on_robot[idx_90].P1.y +
  //      lt_on_robot[idx_0].P2.y) /
  //     3;
  // mark.field_points.back().weight = Measurement::getWeight(
  //     30, dancer_geometry::GetDistance(mark.field_points.back().pred),
  //     parameters.amcl.trust_dist, parameters.amcl.max_dist);
  // for (int ii = -1; ii < 1; ii += 2)
  //   for (int jj = -1; jj < 1; jj += 2)
  //     mark.field_points.back().ref.emplace_back(ii * ref_pt.x, jj *
  //     ref_pt.y);
}
}  // namespace dvision

/* Copyright (C) ZJUDancer
 * 2018 - Yusu Pan <xxdsox@gmail.com>
 * 2017 - Wenxing Mei <mwx36mwx@gmail.com>
 */

/**
 * @file utils.cpp
 * @author Yusu Pan, Wenxing Mei
 * @version 2018
 * @date 2018-02-24
 */

// TODO(MWX, yuthon): use cv::Point2d or cv::Point2f only

#include "dancer_geometry/utils.hpp"

namespace dancer_geometry {
// anything that avoids division overflow
#define SMALL_NUM 0.00000001
// dot product (3D) which allows std::vector operations in arguments
// distance = norm of difference
#define d(u, v) norm(u - v)
#define sign(a) (((a) < 0) ? -1 : ((a) > 0))

double dot(cv::Point3f c1, cv::Point3f c2) {
  return (c1.x * c2.x + c1.y * c2.y + c1.z * c2.z);
}

cv::Point2f RotateCoordinateAxis(const double& alpha, const cv::Point2f& p) {
  double alpha_rad = Degree2Radian(alpha);
  return cv::Point2f(
      static_cast<float>(p.x * std::cos(alpha_rad) - p.y * std::sin(alpha_rad)),
      static_cast<float>(p.x * std::sin(alpha_rad) +
                         p.y * std::cos(alpha_rad)));
}

void RotateCoordinateAxis(const double& alpha, const cv::Point2d& p,
                          cv::Point2d& res) {
  double alpha_rad = Degree2Radian(alpha);
  res = cv::Point2d(
      static_cast<float>(p.x * std::cos(alpha_rad) - p.y * std::sin(alpha_rad)),
      static_cast<float>(p.x * std::sin(alpha_rad) +
                         p.y * std::cos(alpha_rad)));
}

cv::Point2f RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha) {
  cv::Point2f pCenter(0, 0);
  cv::Point2f p = pQuery - pCenter;
  return RotateCoordinateAxis(-alpha, p) + pCenter;
}

cv::Point2f RotateAroundPoint(const cv::Point2f& pQuery, const double& alpha,
                              const cv::Point2f& pCenter) {
  cv::Point2f p = pQuery - pCenter;
  return RotateCoordinateAxis(-alpha, p) + pCenter;
}

void RotateAroundPoint(const cv::Point2d& pQuery, const double& alpha,
                       cv::Point2d& res) {
  cv::Point2d pCenter(0, 0);
  cv::Point2d p = pQuery - pCenter;
  cv::Point2d resRot;
  RotateCoordinateAxis(-alpha, p, resRot);
  res = resRot + pCenter;
}

double Radian2Degree(const double& r) { return (r / M_PI) * (180); }

double Degree2Radian(const double& d) { return (d * M_PI) / (180); }

void LowPass(const cv::Point3f& newrec, cv::Point3f& res, const float& coef) {
  float coefIn = 1 - coef;
  res.x = (coefIn * res.x) + (coef * newrec.x);
  res.y = (coefIn * res.y) + (coef * newrec.y);
  res.z = (coefIn * res.z) + (coef * newrec.z);
}

void LowPass(const cv::Point2f& newrec, cv::Point2f& res, const float& coef) {
  float coefIn = 1 - coef;
  res.x = (coefIn * res.x) + (coef * newrec.x);
  res.y = (coefIn * res.y) + (coef * newrec.y);
}

void LowPass(const float& newrec, float& res, const float& coef) {
  float coefIn = 1 - coef;
  res = (coefIn * res) + (coef * newrec);
}

double GetDistance(const cv::Point2d& p) {
  return std::sqrt((p.x * p.x) + (p.y * p.y));
}

float GetDistance(const cv::Point2f& p) {
  return std::sqrt((p.x * p.x) + (p.y * p.y));
}

double GetDistance(const cv::Point2d& p, const cv::Point2d& p2) {
  double x = std::abs(p.x - p2.x);
  double y = std::abs(p.y - p2.y);
  return std::sqrt(x * x + y * y);
}

float GetDistance(const cv::Point2f& p, const cv::Point2f& p2) {
  float x = std::abs(p.x - p2.x);
  float y = std::abs(p.y - p2.y);
  return std::sqrt(x * x + y * y);
}

float GetDistance(const cv::Point& p, const cv::Point& p2) {
  float x = std::abs(p.x - p2.x);
  float y = std::abs(p.y - p2.y);
  return std::sqrt(x * x + y * y);
}

int Top(const cv::Rect& rec) { return rec.y; }

int Bottom(const cv::Rect& rec) { return rec.y + rec.height; }

int Left(const cv::Rect& rec) { return rec.x; }

int Right(const cv::Rect& rec) { return rec.x + rec.width; }

cv::Scalar grayWhite() { return cv::Scalar(255); }

cv::Scalar pinkColor() { return cv::Scalar(255, 0, 255); }

cv::Scalar pinkMeloColor() { return cv::Scalar(183, 50, 130); }

cv::Scalar whiteColor() { return cv::Scalar(255, 255, 255); }

cv::Scalar redColor() { return cv::Scalar(0, 0, 255); }

cv::Scalar magentaNormalColor() { return cv::Scalar(246, 23, 247); }

cv::Scalar orangeColor() { return cv::Scalar(0, 117, 255); }

cv::Scalar darkOrangeColor() { return cv::Scalar(48, 163, 201); }

cv::Scalar redMeloColor() { return cv::Scalar(51, 123, 255); }

cv::Scalar greenColor() { return cv::Scalar(0, 255, 0); }

cv::Scalar yellowColor() { return cv::Scalar(0, 255, 255); }

cv::Scalar blueColor() { return cv::Scalar(255, 0, 0); }

cv::Scalar blueMeloColor() { return cv::Scalar(255, 153, 51); }

cv::Scalar blueNormalColor() { return cv::Scalar(214, 131, 66); }

cv::Scalar blueLightColor() { return cv::Scalar(243, 176, 108); }

cv::Scalar blueGrayColor() { return cv::Scalar(141, 108, 77); }

cv::Scalar blackColor() { return cv::Scalar(0, 0, 0); }

cv::Scalar blackGary() { return cv::Scalar(0); }

bool MergeLinesMax(std::vector<LineSegment> resLinesReal,
                   const double& maxDegree, const double& maxDistance,
                   const double& minCollinearLength,
                   std::vector<LineSegment>& clusteredLines,
                   const cv::Rect& box, const bool& useBounding) {
  if (!MergeLinesOnce(resLinesReal, maxDegree, maxDistance, minCollinearLength,
                      clusteredLines, box, useBounding)) {
    return false;
  }
  std::vector<LineSegment> before;
  before.reserve(resLinesReal.size());
  do {
    before = clusteredLines;
    if (!MergeLinesOnce(before, maxDegree, maxDistance, minCollinearLength,
                        clusteredLines, box, useBounding)) {
      return false;
    }
  } while (before.size() > clusteredLines.size());
  return true;
}

bool MergeLinesOnce(std::vector<LineSegment> resLinesReal,
                    const double& maxDegree, const double& maxDistance,
                    const double& minCollinearLength,
                    std::vector<LineSegment>& clusteredLines,
                    const cv::Rect& box, const bool& useBounding) {
  if (resLinesReal.size() < 1) {
    return false;
  }
  clusteredLines.clear();
  clusteredLines.reserve(resLinesReal.size());
  std::vector<cv::Point2f> pointVector(4);
  for (size_t i = 0; i < resLinesReal.size(); i++) {
    int index = -1;
    for (size_t j = 0; j < clusteredLines.size(); j++) {
      double diffAngle =
          resLinesReal[i].GetAbsMinAngleDegree(clusteredLines[j]);
      bool parallelOK = (diffAngle < maxDegree) &&
                        (dist3D_Segment_to_Segment(
                             resLinesReal[i], clusteredLines[j]) < maxDistance);
      if (parallelOK) {
        // cout << "-- -- >   BGN Merge  < -- --" << endl;
        // cout << "DiffAngle = " << diffAngle << endl;
        // cout << "DiffDist  = " << dist3D_Segment_to_Segment(resLinesReal[i],
        // clusteredLines[j]) << endl; cout << " line  = " << resLinesReal[i].P1
        // << " " << resLinesReal[i].P2 << endl; cout << " cLine = " <<
        // clusteredLines[j].P1 << " " << clusteredLines[j].P2 << endl;
        index = j;
        break;
      } else {
        LineSegment midLine(resLinesReal[i].GetMiddle(),
                            clusteredLines[j].GetMiddle());
        double diffAngleMid1 = midLine.GetAbsMinAngleDegree(resLinesReal[i]);
        double diffAngleMid2 = midLine.GetAbsMinAngleDegree(clusteredLines[j]);
        double lineLength1 = resLinesReal[i].GetLength();
        double lineLength2 = clusteredLines[j].GetLength();
        bool collinearOK =
            (diffAngleMid1 < maxDegree / 2) &&
            (diffAngleMid2 < maxDegree / 2) &&
            (lineLength1 > minCollinearLength) &&
            (lineLength2 > minCollinearLength) &&
            (dist3D_Segment_to_Segment(resLinesReal[i], clusteredLines[j]) <
             maxDistance * 5);
        if (collinearOK) {
          index = j;
          break;
        } else {
        }
        // cout << "DiffAngle = " << diffAngle << endl;
        // cout << "DiffDist  = " << dist3D_Segment_to_Segment(resLinesReal[i],
        // clusteredLines[j]) << endl; cout << " line  = " << resLinesReal[i].P1
        // << " " << resLinesReal[i].P2 << endl; cout << " cLine = " <<
        // clusteredLines[j].P1 << " " << clusteredLines[j].P2 << endl;
      }
    }

    if (index != -1) {
      // existed
      cv::Point2f mergedMidPoint;
      LineSegment mergedLine;

      LineSegment queryLine1(clusteredLines[index]);
      LineSegment queryLine2(resLinesReal[i]);

      pointVector[0] = (queryLine1.P1);
      pointVector[1] = (queryLine1.P2);

      pointVector[2] = (queryLine2.P1);
      pointVector[3] = (queryLine2.P2);
      float queryLineLen1 = queryLine1.GetLength();
      float queryLineLen2 = queryLine2.GetLength();
      if (!useBounding) {
        float angle1 = queryLine1.GetRadianFromX();
        float angle2 = queryLine2.GetRadianFromX();
        mergedMidPoint =
            GetWeightedAverage(queryLine1.GetMiddle(), queryLine2.GetMiddle(),
                               queryLineLen1, queryLineLen2);
        float mergedAngle = CorrectAngleRadian180(
            (queryLineLen1 > queryLineLen2) ? angle1 : angle2);

        // topViewBox.x = -1 * params.topView->width.get();     //-900
        // topViewBox.y = -1 * params.topView->width.get();     //-900
        // topViewBox.width = 2 * params.topView->width.get();  // 1800
        // topViewBox.height = 2 * params.topView->width.get(); // 1800

        double theMult = std::max(box.width - box.x, box.height - box.y);
        LineSegment biggestDistanceLS;
        mergedLine.P1.x = mergedMidPoint.x - theMult * std::cos(mergedAngle);
        mergedLine.P1.y = mergedMidPoint.y - theMult * std::sin(mergedAngle);
        mergedLine.P2.x = mergedMidPoint.x + theMult * std::cos(mergedAngle);
        mergedLine.P2.y = mergedMidPoint.y + theMult * std::sin(mergedAngle);

        // Find max distance points on the new line
        double maxDistance = -9999999;
        for (int i = 0; i < 4; i++) {
          for (int j = i + 1; j < 4; j++) {
            cv::Point2f p1 =
                mergedLine.GetClosestPointOnLineSegment(pointVector[i]);
            cv::Point2f p2 =
                mergedLine.GetClosestPointOnLineSegment(pointVector[j]);
            double distance = LineSegment(p1, p2).GetLength();
            if (distance > maxDistance) {
              maxDistance = distance;
              biggestDistanceLS.P1 = p1;
              biggestDistanceLS.P2 = p2;
            }
          }
        }
        mergedLine = biggestDistanceLS;
      } else {
        cv::RotatedRect minRec = minAreaRect(pointVector);
        cv::Point2f allPoints[4];
        minRec.points(allPoints);
        if (LineSegment(allPoints[2], allPoints[1]).GetLength() >
            LineSegment(allPoints[2], allPoints[3]).GetLength()) {
          mergedLine.P1 = GetAverage(allPoints[2], allPoints[3]);
          mergedLine.P2 = GetAverage(allPoints[0], allPoints[1]);
        } else {
          mergedLine.P1 = GetAverage(allPoints[2], allPoints[1]);
          mergedLine.P2 = GetAverage(allPoints[0], allPoints[3]);
        }
      }

      mergedLine.SetProbability((queryLine1.GetProbability() * queryLineLen1 +
                                 queryLine2.GetProbability() * queryLineLen2) /
                                (queryLineLen1 + queryLineLen2));
      mergedLine.Clip(box);
      clusteredLines[index].P1 = cv::Point2f(mergedLine.P1.x, mergedLine.P1.y);
      clusteredLines[index].P2 = cv::Point2f(mergedLine.P2.x, mergedLine.P2.y);
      // std::cout << "mergedLine.GetProbability(): " <<
      // mergedLine.GetProbability() << std::endl;
      // clusteredLines[index].SetProbability(mergedLine.GetProbability());

      // cout << " Merged Line = " << mergedLine.P1 << "  " << mergedLine.P2 <<
      // endl; cout << "-- -- >      END     < -- --" << endl;
    } else {
      // 如果不存在要merge的线，则push到cluster
      clusteredLines.push_back(resLinesReal[i]);
    }
  }
  return true;
}

float dist3D_Segment_to_Segment(const LineSegment& S1, const LineSegment& S2) {
  cv::Point3f u;
  u.x = S1.P2.x - S1.P1.x;
  u.y = S1.P2.y - S1.P1.y;
  u.z = 0;

  cv::Point3f v;
  v.x = S2.P2.x - S2.P1.x;
  v.y = S2.P2.y - S2.P1.y;
  v.z = 0;
  cv::Point3f w;
  w.x = S1.P1.x - S2.P1.x;
  w.y = S1.P1.y - S2.P1.y;
  w.z = 0;

  // always >= 0
  float a = dot(u, u);
  float b = dot(u, v);
  // always >= 0
  float c = dot(v, v);
  float d = dot(u, w);
  float e = dot(v, w);
  // always >= 0
  float D = a * c - b * b;
  // sc = sN / sD, default sD = D >= 0
  float sc, sN, sD = D;
  // tc = tN / tD, default tD = D >= 0
  float tc, tN, tD = D;

  // compute the line parameters of the two closest points
  if (D < SMALL_NUM) {
    // the lines are almost parallel
    // force using point P0 on segment S1
    sN = 0.0;
    // to prevent possible division by 0.0 later
    sD = 1.0;
    tN = e;
    tD = c;
  } else {
    // get the closest points on the infinite lines
    sN = (b * e - c * d);
    tN = (a * e - b * d);
    if (sN < 0.0) {
      // sc < 0 => the s=0 edge is visible
      sN = 0.0;
      tN = e;
      tD = c;
    } else if (sN > sD) {
      // sc > 1  => the s=1 edge is visible
      sN = sD;
      tN = e + b;
      tD = c;
    }
  }

  if (tN < 0.0) {
    // tc < 0 => the t=0 edge is visible
    tN = 0.0;
    // recompute sc for this edge
    if (-d < 0.0) {
      sN = 0.0;
    } else if (-d > a) {
      sN = sD;
    } else {
      sN = -d;
      sD = a;
    }
  } else if (tN > tD) {
    // tc > 1  => the t=1 edge is visible
    tN = tD;
    // recompute sc for this edge
    if ((-d + b) < 0.0) {
      sN = 0;
    } else if ((-d + b) > a) {
      sN = sD;
    } else {
      sN = (-d + b);
      sD = a;
    }
  }
  // finally do the division to get sc and tc
  sc = (std::abs(sN) < SMALL_NUM ? 0.0 : sN / sD);
  tc = (std::abs(tN) < SMALL_NUM ? 0.0 : tN / tD);

  // get the difference of the two closest points
  // =  S1(sc) - S2(tc)
  cv::Point3d dP = w + (sc * u) - (tc * v);

  // return the closest distance
  return norm(dP);
}

// Normalize to [0,360):
double CorrectAngleDegree360(const double& x) {
  double x_mod = fmod(x, 360);
  if (x_mod < 0) {
    x_mod += 360;
  }
  return x_mod;
}

// Normalize to [-180,180)
double CorrectAngleDegree180(const double& x) {
  double x_mod = fmod(x + 180, 360);
  if (x_mod < 0) {
    x_mod += 360;
  }
  return x_mod - 180;
}

// Normalize to [0,360):
double CorrectAngleRadian360(const double& x) {
  return Degree2Radian((CorrectAngleDegree360((Radian2Degree(x)))));
}

// Normalize to [-180,180)
double CorrectAngleRadian180(const double& x) {
  return Degree2Radian((CorrectAngleDegree180((Radian2Degree(x)))));
}

// Normalize to [-180,180)
float AngleDiffDegree180(const float& first, const float& second) {
  return CorrectAngleDegree180(first - second);
}

// Normalize to [-180,180)
float AngleDiffRadian180(const float& first, const float& second) {
  return CorrectAngleRadian180(first - second);
}

cv::Point2f GetAverage(const cv::Point2f& p0, const cv::Point2f& p1) {
  return cv::Point2f((p0.x + p1.x) / 2., (p0.y + p1.y) / 2.);
}

cv::Point2f GetWeightedAverage(const cv::Point2f& p0, const cv::Point2f& p1,
                               const float& w0, const float& w1) {
  float sumW = w0 + w1;
  return cv::Point2f((p0.x * w0 + p1.x * w1) / sumW,
                     (p0.y * w0 + p1.y * w1) / sumW);
}

float GetWeightedAverage(const float& p0, const float& p1, const float& w0,
                         const float& w1) {
  float sumW = w0 + w1;
  return (p0 * w0 + p1 * w1) / sumW;
}

// point distance from line segment
float DistanceFromLineSegment(const LineSegment& line, const cv::Point2f& p) {
  cv::Point2d segA = line.P1;
  cv::Point2d segB = line.P2;

  cv::Point2d p2(segB.x - segA.x, segB.y - segA.y);

  float something = p2.x * p2.x + p2.y * p2.y;
  float u = ((p.x - segA.x) * p2.x + (p.y - segA.y) * p2.y) / something;

  if (u > 1) {
    u = 1;
  } else if (u < 0) {
    u = 0;
  }

  float x = segA.x + u * p2.x;
  float y = segA.y + u * p2.y;

  float dx = x - p.x;
  float dy = y - p.y;

  float dist = static_cast<float>(std::sqrt(dx * dx + dy * dy));

  return dist;
}

bool SortFuncDescending(const std::vector<cv::Point>& i,
                        const std::vector<cv::Point>& j) {
  return cv::contourArea(i, false) > cv::contourArea(j, false);
}

std::vector<LineSegment> getOnGlobalCoordinate(
    const cv::Point3d& robot_pos, const std::vector<LineSegment>& in_lines) {
  std::vector<LineSegment> out_lines(in_lines.size());
  for (size_t i = 0; i < in_lines.size(); i++) {
    out_lines[i] = LineSegment(getOnGlobalCoordinate(robot_pos, in_lines[i].P1),
                               getOnGlobalCoordinate(robot_pos, in_lines[i].P2),
                               in_lines[i].GetProbability());
  }
  return out_lines;
}

std::vector<cv::Point2f> getOnGlobalCoordinate(
    const cv::Point3d& robot_pos, const std::vector<cv::Point2f>& in_points) {
  std::vector<cv::Point2f> out_points(in_points.size());
  for (size_t i = 0; i < in_points.size(); i++) {
    out_points[i] = getOnGlobalCoordinate(robot_pos, in_points[i]);
  }
  return out_points;
}

cv::Point2f getOnGlobalCoordinate(const cv::Point3d& robot_pos,
                                  const cv::Point2f& in_point) {
  cv::Point2f out_point;
  out_point = RotateAroundPoint(in_point, -robot_pos.z);
  out_point.x += robot_pos.x;
  out_point.y += robot_pos.y;
  return out_point;
}

cv::Point2f getOnGlobalCoordinate(const cv::Point3d& robot_pos, float x,
                                  float y) {
  return getOnGlobalCoordinate(robot_pos, cv::Point2f(x, y));
}

cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  float x, float y) {
  return getOnGlobalCoordinate(
      cv::Point3d(robot_pos.x, robot_pos.y, robot_pos.z), x, y);
}

cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  const cv::Point2f& in_point) {
  cv::Point2f out_point;
  out_point = RotateAroundPoint(in_point, -robot_pos.z);
  out_point.x += robot_pos.x;
  out_point.y += robot_pos.y;
  return out_point;
}

cv::Point2f getOnGlobalCoordinate(const geometry_msgs::Vector3& robot_pos,
                                  const geometry_msgs::Vector3& in_point) {
  return getOnGlobalCoordinate(robot_pos, in_point.x, in_point.y);
}

cv::Point2d getOnGlobalCoordinate(const cv::Point3d& robot_pos,
                                  const cv::Point2d& in_point) {
  cv::Point2d out_point;
  out_point = RotateAroundPoint(in_point, -robot_pos.z);
  out_point.x += robot_pos.x;
  out_point.y += robot_pos.y;
  return out_point;
}
std::vector<double> getOnGlobalCoordinate(
    const std::vector<double>& robot_pos_v,
    const std::vector<double>& in_point_v) {
  cv::Point3d robot_pos(robot_pos_v[0], robot_pos_v[1], robot_pos_v[2]);
  cv::Point2d in_point(in_point_v[0], in_point_v[1]);
  cv::Point2d out_point;
  out_point = RotateAroundPoint(in_point, -robot_pos.z);
  out_point.x += robot_pos.x;
  out_point.y += robot_pos.y;
  return {out_point.x, out_point.y};
}

cv::Point2d getOnRobotCoordinate(const cv::Point3d& robot_pos,
                                 const cv::Point2d& in_point) {
  cv::Point2d out_point = in_point;
  out_point.x -= robot_pos.x;
  out_point.y -= robot_pos.y;
  out_point = RotateAroundPoint(out_point, robot_pos.z);
  return out_point;
}

std::vector<double> getOnRobotCoordinate(
    const std::vector<double>& robot_pos_v,
    const std::vector<double>& in_point_v) {
  cv::Point3d robot_pos(robot_pos_v[0], robot_pos_v[1], robot_pos_v[2]);
  cv::Point2d in_point(in_point_v[0], in_point_v[1]);
  cv::Point2d out_point = in_point;
  out_point.x -= robot_pos.x;
  out_point.y -= robot_pos.y;
  out_point = RotateAroundPoint(out_point, robot_pos.z);
  return {out_point.x, out_point.y};
}

}  // namespace dancer_geometry

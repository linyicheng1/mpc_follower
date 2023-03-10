/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mpc_utils.h"

void MPCUtils::convertEulerAngleToMonotonic(std::vector<double> &a)
{
  for (unsigned int i = 1; i < a.size(); ++i)
  {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + radianNormalize(da);
  }
}

template <typename T1, typename T2>
bool MPCUtils::interp1d(const T1 &index, const T2 &values, const double &ref, double &ret)
{
  ret = 0.0;
  if (!((int)index.size() == (int)values.size()))
  {
    printf("index and values must have same size, return false. size : idx = %d, values = %d\n", (int)index.size(), (int)values.size());
    return false;
  }
  if (index.size() == 1)
  {
    printf("index size is 1, too short. return false.\n");
    return false;
  }
  unsigned int end = index.size() - 1;
  if (ref < index[0])
  {
    ret = values[0];
    // printf("ref point is out of index (low), return false.\n");
    return true;
  }
  if (index[end] < ref)
  {
    ret = values[end];
    // printf("ref point is out of index (high), return false.\n");
    return true;
  }

  for (unsigned int i = 1; i < index.size(); ++i)
  {
    if (!(index[i] > index[i - 1]))
    {
      printf("index must be monotonically increasing, return false. index[%d] = %f, but index[%d] = %f\n", i, index[i], i-1, index[i - 1]);
      return false;
    }
  }
  unsigned int i = 1;
  while (ref > index[i])
  {
    ++i;
  }
  const double a = ref - index[i - 1];
  const double d_index = index[i] - index[i - 1];
  ret = ((d_index - a) * values[i - 1] + a * values[i]) / d_index;
  return true;
}
template bool MPCUtils::interp1d<std::vector<double>, std::vector<double>>(const std::vector<double> &, const std::vector<double> &, const double &, double &);
template bool MPCUtils::interp1d<std::vector<double>, Eigen::VectorXd>(const std::vector<double> &, const Eigen::VectorXd &, const double &, double &);
template bool MPCUtils::interp1d<Eigen::VectorXd, std::vector<double>>(const Eigen::VectorXd &, const std::vector<double> &, const double &, double &);
template bool MPCUtils::interp1d<Eigen::VectorXd, Eigen::VectorXd>(const Eigen::VectorXd &, const Eigen::VectorXd &, const double &, double &);

// 1D interpolation
bool MPCUtils::interp1dMPCTraj(const std::vector<double> &index, const MPCTrajectory &values,
                               const std::vector<double> &ref_time, MPCTrajectory &ret)
{
  if (!(index.size() == values.size()))
  {
    printf("index and values must have same size, return false.\n");
    return false;
  }
  if (index.size() == 1)
  {
    printf("index size is 1, too short. return false.\n");
    return false;
  }

  for (unsigned int i = 1; i < index.size(); ++i)
  {
    if (!(index[i] > index[i - 1]))
    {
      printf("index must be monotonically increasing, return false. index[%d] = %f, but index[%d] = %f\n", i, index[i], i-1, index[i - 1]);
      return false;
    }
  }

  for (unsigned int i = 1; i < ref_time.size(); ++i)
  {
    if (!(ref_time[i] > ref_time[i - 1]))
    {
      printf("reference point must be monotonically increasing, return false. ref_time[%d] = %f, but ref_time[%d] = %f\n", i, ref_time[i], i-1, ref_time[i - 1]);
      return false;
    }
  }

  ret.clear();
  unsigned int i = 1;
  for (unsigned int j = 0; j < ref_time.size(); ++j)
  {
    double a, d_index;
    if (ref_time[j] > index.back())
    {
      a = 1.0;
      d_index = 1.0;
      i = index.size() - 1;
    }
    else if (ref_time[j] < index.front())
    {
      a = 0.0;
      d_index = 1.0;
      i = 1;
    }
    else
    {
      while (ref_time[j] > index[i])
      {
        ++i;
      }
      a = ref_time[j] - index[i - 1];
      d_index = index[i] - index[i - 1];
    }
    const double x = ((d_index - a) * values.x[i - 1] + a * values.x[i]) / d_index;
    const double y = ((d_index - a) * values.y[i - 1] + a * values.y[i]) / d_index;
    const double z = ((d_index - a) * values.z[i - 1] + a * values.z[i]) / d_index;
    const double yaw = ((d_index - a) * values.yaw[i - 1] + a * values.yaw[i]) / d_index;
    const double vx = ((d_index - a) * values.vx[i - 1] + a * values.vx[i]) / d_index;
    const double k = ((d_index - a) * values.k[i - 1] + a * values.k[i]) / d_index;
    const double t = ref_time[j];
    ret.push_back(x, y, z, yaw, vx, k, t);
  }
  return true;
}

void MPCUtils::calcTrajectoryYawFromXY(MPCTrajectory &traj)
{
  if (traj.yaw.size() == 0)
    return;

  for (unsigned int i = 1; i < traj.yaw.size() - 1; ++i)
  {
    const double dx = traj.x[i + 1] - traj.x[i - 1];
    const double dy = traj.y[i + 1] - traj.y[i - 1];
    traj.yaw[i] = std::atan2(dy, dx);
  }
  if (traj.yaw.size() > 1)
  {
    traj.yaw[0] = traj.yaw[1];
    traj.yaw.back() = traj.yaw[traj.yaw.size() - 2];
  }
}

void MPCUtils::calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num)
{
  unsigned int traj_k_size = traj.x.size();
  traj.k.clear();

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  for (unsigned int i = curvature_smoothing_num; i < traj_k_size - curvature_smoothing_num; ++i)
  {
    p1.x = traj.x[i - curvature_smoothing_num];
    p2.x = traj.x[i];
    p3.x = traj.x[i + curvature_smoothing_num];
    p1.y = traj.y[i - curvature_smoothing_num];
    p2.y = traj.y[i];
    p3.y = traj.y[i + curvature_smoothing_num];
    double den = std::max(find_distance(p1, p2) * find_distance(p2, p3) * find_distance(p3, p1), 0.0001);
    const double curvature = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    traj.k.push_back(curvature);
  }

  /* first and last curvature is copied from next value */
  for (int i = 0; i < curvature_smoothing_num; ++i)
  {
    traj.k.insert(traj.k.begin(), traj.k.front());
    traj.k.push_back(traj.k.back());
  }
}

void MPCUtils::convertWaypointsToMPCTraj(const mpc_follower::MPCPath &lane, MPCTrajectory &mpc_traj)
{
  mpc_traj.clear();
  const int size = lane.x.size();
  for (int i = 0; i < size; i++)
  {
    mpc_traj.push_back(lane.x[i], lane.y[i], 0, lane.yaw[i], lane.vx[i], lane.k[i], lane.relative_time[i]);
  }
}

void MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(const mpc_follower::MPCPath &path, const std::vector<double> &path_time,
                                                             const double &dl, MPCTrajectory &ref_traj)
{
  ref_traj.clear();
  double dist = 0.0;
  std::vector<double> dists;
  dists.push_back(0.0);

  for (int i = 1; i < (int)path_time.size(); ++i)
  {
    double dx = path.x.at(i) - path.x.at(i - 1);
    double dy = path.y.at(i) - path.y.at(i - 1);
    dist += sqrt(dx * dx + dy * dy);
    dists.push_back(dist);
  }

  convertWaypointsToMPCTrajWithResample(path, path_time, dists, dl, ref_traj);
}


void MPCUtils::convertWaypointsToMPCTrajWithTimeResample(const mpc_follower::MPCPath &path, const std::vector<double> &path_time,
                                                         const double &dt, MPCTrajectory &ref_traj)
{
  ref_traj.clear();
  convertWaypointsToMPCTrajWithResample(path, path_time, path_time, dt, ref_traj);
}

void MPCUtils::convertWaypointsToMPCTrajWithResample(const mpc_follower::MPCPath &path, const std::vector<double> &path_time,
                                                     const std::vector<double> &ref_index, const double &d_ref_index, MPCTrajectory &ref_traj)
{
  if (ref_index.size() == 0) {
    return;
  }

  for (unsigned int i = 1; i < ref_index.size(); ++i)
  {
    if (ref_index[i] < ref_index[i - 1])
    {
      ROS_ERROR("[convertWaypointsToMPCTrajWithResample] resampling index must be monotonically increasing. idx[%d] = %f, idx[%d+1] = %f",
                i, ref_index[i], i, ref_index[i + 1]);
      return;
    }
  }

  double point = ref_index[0];
  while (point < ref_index.back())
  {
    unsigned int j = 1;
    while (point > ref_index.at(j))
    {
      ++j;
    }

    const double a = point - ref_index.at(j - 1);
    const double ref_index_dist = ref_index.at(j) - ref_index.at(j - 1);
//    const geometry_msgs::Pose pos0 = path.waypoints.at(j - 1).pose.pose;
//    const geometry_msgs::Pose pos1 = path.waypoints.at(j).pose.pose;
//    const geometry_msgs::Twist twist0 = path.waypoints.at(j - 1).twist.twist;
//    const geometry_msgs::Twist twist1 = path.waypoints.at(j).twist.twist;
//    const double x = ((ref_index_dist - a) * pos0.position.x + a * pos1.position.x) / ref_index_dist;
//    const double y = ((ref_index_dist - a) * pos0.position.y + a * pos1.position.y) / ref_index_dist;
//    const double z = ((ref_index_dist - a) * pos0.position.z + a * pos1.position.z) / ref_index_dist;

    /* for singular point of euler angle */
//    const double yaw0 = tf2::getYaw(pos0.orientation);
//    const double dyaw = radianNormalize(tf2::getYaw(pos1.orientation) - yaw0);
//    const double yaw1 = yaw0 + dyaw;
//    const double yaw = ((ref_index_dist - a) * yaw0 + a * yaw1) / ref_index_dist;
//    const double vx = ((ref_index_dist - a) * twist0.linear.x + a * twist1.linear.x) / ref_index_dist;
//    const double curvature_tmp = 0.0;
//    const double t = ((ref_index_dist - a) * path_time.at(j - 1) + a * path_time.at(j)) / ref_index_dist;
//    ref_traj.push_back(x, y, z, yaw, vx, curvature_tmp, t);
//    point += d_ref_index;
  }
}

void MPCUtils::calcPathRelativeTime(const mpc_follower::MPCPath &path, std::vector<double> &path_time)
{
  double t = 0.0;
  path_time.clear();
  path_time.push_back(t);
  const int size = path.x.size();
  for (int i = 0; i < size - 1; ++i)
  {
    const double x0 = path.x.at(i);
    const double y0 = path.y.at(i);
    const double z0 = 0;
    const double x1 = path.x.at(i + 1);
    const double y1 = path.y.at(i + 1);
    const double z1 = 0;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dz = z1 - z0;
    const double dist = sqrt(dx * dx + dy * dy + dz * dz);
    double v = std::max(std::fabs(path.vx.at(i)), 1.0);
    t += (dist / v);
    path_time.push_back(t);
  }
}

bool MPCUtils::calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                               unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time)
{
  int nearest_index_tmp = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  nearest_yaw_error = std::numeric_limits<double>::max();
  for (uint i = 0; i < traj.size(); ++i)
  {
    const double dx = self_pose.position.x - traj.x[i];
    const double dy = self_pose.position.y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = radianNormalize(tf2::getYaw(self_pose.orientation) - traj.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 3.0))
    {
      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_yaw_error = err_yaw;
        nearest_index_tmp = i;
      }
    }
  }
  if (nearest_index_tmp == -1)
  {
    ROS_WARN("[calcNearestPose] yaw error is over PI/3 for all waypoints. no closest waypoint found.");
    return false;
  }

  nearest_index = nearest_index_tmp;

  min_dist_error = std::sqrt(min_dist_squared);
  nearest_time = traj.relative_time[nearest_index];
  nearest_pose.position.x = traj.x[nearest_index];
  nearest_pose.position.y = traj.y[nearest_index];
  // TODO
//  nearest_pose.orientation = amathutils::getQuaternionFromYaw(traj.yaw[nearest_index]);
  return true;
};

bool MPCUtils::calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                                     unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time)
{

  if (traj.size() == 0)
  {
    ROS_WARN("[calcNearestPoseInterp] trajectory size is zero");
    return false;
  }
  const double my_x = self_pose.position.x;
  const double my_y = self_pose.position.y;
  const double my_yaw = tf2::getYaw(self_pose.orientation);

  int nearest_index_tmp = -1;
  double min_dist_squared = std::numeric_limits<double>::max();
  for (uint i = 0; i < traj.size(); ++i)
  {
    const double dx = my_x - traj.x[i];
    const double dy = my_y - traj.y[i];
    const double dist_squared = dx * dx + dy * dy;

    /* ignore when yaw error is large, for crossing path */
    const double err_yaw = radianNormalize(my_yaw - traj.yaw[i]);
    if (fabs(err_yaw) < (M_PI / 3.0))
    {
      if (dist_squared < min_dist_squared)
      {
        /* save nearest index */
        min_dist_squared = dist_squared;
        nearest_index_tmp = i;
      }
    }
  }
  if (nearest_index_tmp == -1)
  {
    ROS_WARN("[calcNearestPoseInterp] yaw error is over PI/3 for all waypoints. no closest waypoint found.");
    return false;
  }

  nearest_index = nearest_index_tmp;

  if (traj.size() == 1)
  {
    nearest_pose.position.x = traj.x[nearest_index];
    nearest_pose.position.y = traj.y[nearest_index];
    tf2::Quaternion q;
    q.setRPY(0, 0, traj.yaw[nearest_index]);
    nearest_pose.orientation = tf2::toMsg(q);
    nearest_time = traj.relative_time[nearest_index];
    min_dist_error = std::sqrt(min_dist_squared);
    nearest_yaw_error = radianNormalize(my_yaw - traj.yaw[nearest_index]);
    return true;
  }

  /* get second nearest index = next to nearest_index */
  int second_nearest_index = 0;
  if (nearest_index == traj.size() - 1)
    second_nearest_index = nearest_index - 1;
  else if (nearest_index == 0)
    second_nearest_index = 1;
  else
  {
    double dx1, dy1, dist_squared1, dx2, dy2, dist_squared2;
    dx1 = my_x - traj.x[nearest_index + 1];
    dy1 = my_y - traj.y[nearest_index + 1];
    dist_squared1 = dx1 * dx1 + dy1 * dy1;
    dx2 = my_x - traj.x[nearest_index - 1];
    dy2 = my_y - traj.y[nearest_index - 1];
    dist_squared2 = dx2 * dx2 + dy2 * dy2;
    if (dist_squared1 < dist_squared2)
      second_nearest_index = nearest_index + 1;
    else
      second_nearest_index = nearest_index - 1;
  }

  const double a_sq = min_dist_squared;

  /* distance between my position and second nearest position */
  const double dx2 = my_x - traj.x[second_nearest_index];
  const double dy2 = my_y - traj.y[second_nearest_index];
  const double b_sq = dx2 * dx2 + dy2 * dy2;

  /* distance between first and second nearest position */
  const double dx3 = traj.x[nearest_index] - traj.x[second_nearest_index];
  const double dy3 = traj.y[nearest_index] - traj.y[second_nearest_index];
  const double c_sq = dx3 * dx3 + dy3 * dy3;

  /* if distance between two points are too close */
  if (c_sq < 1.0E-5)
  {
    nearest_pose.position.x = traj.x[nearest_index];
    nearest_pose.position.y = traj.y[nearest_index];
    tf2::Quaternion q;
    q.setRPY(0, 0, traj.yaw[nearest_index]);
    nearest_pose.orientation = tf2::toMsg(q);
    nearest_time = traj.relative_time[nearest_index];
    min_dist_error = std::sqrt(min_dist_squared);
    nearest_yaw_error = radianNormalize(my_yaw - traj.yaw[nearest_index]);
    return true;
  }

  /* linear interpolation */
  const double alpha = 0.5 * (c_sq - a_sq + b_sq) / c_sq;
  nearest_pose.position.x = alpha * traj.x[nearest_index] + (1 - alpha) * traj.x[second_nearest_index];
  nearest_pose.position.y = alpha * traj.y[nearest_index] + (1 - alpha) * traj.y[second_nearest_index];
  double tmp_yaw_err = traj.yaw[nearest_index] - traj.yaw[second_nearest_index];
  if (tmp_yaw_err > M_PI)
  {
    tmp_yaw_err -= 2.0 * M_PI;
  }
  else if (tmp_yaw_err < -M_PI)
  {
    tmp_yaw_err += 2.0 * M_PI;
  }
  const double nearest_yaw = traj.yaw[second_nearest_index] + alpha * tmp_yaw_err;
  tf2::Quaternion q;
  q.setRPY(0, 0, nearest_yaw);
  nearest_pose.orientation = tf2::toMsg(q);
  nearest_time = alpha * traj.relative_time[nearest_index] + (1 - alpha) * traj.relative_time[second_nearest_index];

  /* calcuate the perpendicular distance from ego position to the line joining
     2 nearest way points. */
  auto min_dist_err_sq = b_sq - c_sq * alpha * alpha;

  /* If ego vehicle is very close to or on the line, min_dist_err_sq would be
     very close to 0, any rounding error in the floating point arithmetic
     could cause it to become negative. Hence its value is limited to 0
     in order to perform sqrt. */
  if (min_dist_err_sq < 0) {
    min_dist_err_sq = 0;
  }

  min_dist_error = std::sqrt(min_dist_err_sq);

  nearest_yaw_error = radianNormalize(my_yaw - nearest_yaw);
  return true;
}

double MPCUtils::radianNormalize(double _angle)
{
    double n_angle = std::fmod(_angle, 2 * M_PI);
    n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;

    // another way
    // Math.atan2(Math.sin(_angle), Math.cos(_angle));
    return n_angle;
}

double MPCUtils::find_distance(const geometry_msgs::Point &_from, const geometry_msgs::Point &_to)
{
    return std::hypot(std::hypot(_from.x - _to.x, _from.y - _to.y), _from.z - _to.z);
}

double MPCUtils::find_distance(const geometry_msgs::Pose &_from, const geometry_msgs::Pose &_to)
{
    return find_distance(_from.position, _to.position);
}
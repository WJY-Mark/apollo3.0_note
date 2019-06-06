/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/common/trajectory/trajectory_stitcher.h"

#include <algorithm>
#include <list>
#include <utility>

#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/common/math/angle.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::VehicleState;
using apollo::common::math::Vec2d;
using apollo::common::util::DistanceXY;

std::vector<TrajectoryPoint>
TrajectoryStitcher::ComputeReinitStitchingTrajectory(
    const VehicleState& vehicle_state) {
  TrajectoryPoint init_point;
  init_point.mutable_path_point()->set_s(0.0);
  init_point.mutable_path_point()->set_x(vehicle_state.x());
  init_point.mutable_path_point()->set_y(vehicle_state.y());
  init_point.mutable_path_point()->set_z(vehicle_state.z());
  init_point.mutable_path_point()->set_theta(vehicle_state.heading());
  init_point.mutable_path_point()->set_kappa(vehicle_state.kappa());
  init_point.set_v(vehicle_state.linear_velocity());
  init_point.set_a(vehicle_state.linear_acceleration());
  init_point.set_relative_time(0.0);

  return std::vector<TrajectoryPoint>(1, init_point);
}

// only used in navigation mode
void TrajectoryStitcher::TransformLastPublishedTrajectory(
    const double x_diff, const double y_diff, const double theta_diff,
    PublishableTrajectory* prev_trajectory) {
  if (!prev_trajectory) {
    return;
  }

  // R^-1
  float cos_theta =
      common::math::cos(common::math::Angle16::from_rad(theta_diff));
  float sin_theta =
      -common::math::sin(common::math::Angle16::from_rad(theta_diff));

  // -R^-1 * t
  auto tx = -(cos_theta * x_diff - sin_theta * y_diff);
  auto ty = -(sin_theta * x_diff + cos_theta * y_diff);

  std::for_each(prev_trajectory->trajectory_points().begin(),
                prev_trajectory->trajectory_points().end(),
                [&cos_theta, &sin_theta, &tx, &ty,
                 &theta_diff](common::TrajectoryPoint& p) {
                  auto x = p.path_point().x();
                  auto y = p.path_point().y();
                  auto theta = p.path_point().theta();

                  auto x_new = cos_theta * x - sin_theta * y + tx;
                  auto y_new = sin_theta * x + cos_theta * y + ty;
                  auto theta_new = common::math::WrapAngle(theta - theta_diff);

                  p.mutable_path_point()->set_x(x_new);
                  p.mutable_path_point()->set_y(y_new);
                  p.mutable_path_point()->set_theta(theta_new);
                });
}

// Planning from current vehicle state:
// if 1. the auto-driving mode is off or
//    2. we don't have the trajectory from last planning cycle or
//    3. the position deviation from actual and target is too high
std::vector<TrajectoryPoint> TrajectoryStitcher::ComputeStitchingTrajectory(
    const VehicleState& vehicle_state, const double current_timestamp,
    const double planning_cycle_time,
    const PublishableTrajectory* prev_trajectory, bool* is_replan) {
  *is_replan = true;
  // 没有启用轨迹缝合功能,规划从车辆当前位置开始
  if (!FLAGS_enable_trajectory_stitcher) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 没有获取到上一周期的轨迹,规划从车辆当前位置开始
  if (!prev_trajectory) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 未处于自动驾驶状态,规划从车辆当前位置开始
  if (vehicle_state.driving_mode() != canbus::Chassis::COMPLETE_AUTO_DRIVE) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 上一周期轨迹点个数
  std::size_t prev_trajectory_size = prev_trajectory->NumOfPoints();

  // 上一周期轨迹点的数量为0, 那么规划起点从车辆的当前位置开始
  if (prev_trajectory_size == 0) {
    ADEBUG << "Projected trajectory at time [" << prev_trajectory->header_time()
           << "] size is zero! Previous planning not exist or failed. Use "
              "origin car status instead.";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  // 在时间上考虑

  // 车辆当前的间戳与上一周期轨迹时间戳的差值
  const double veh_rel_time =
      current_timestamp - prev_trajectory->header_time();

  // 从上一周期轨迹中查找当前车辆状态时间戳对应的轨迹点的id
  std::size_t time_matched_index =
      prev_trajectory->QueryLowerBoundPoint(veh_rel_time);

  // 如果车辆当前还没有到达上一周期路径的起点，那么当前周期规划从车辆当前位置开始
  if (time_matched_index == 0 &&
      veh_rel_time < prev_trajectory->StartPoint().relative_time()) {
    AWARN << "current time smaller than the previous trajectory's first time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  // 如果车辆当前已经超过了上一周期路径的末点，那么当前周期规划从车辆当前位置开始
  if (time_matched_index + 1 >= prev_trajectory_size) {
    AWARN << "current time beyond the previous trajectory's last time";
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  
  // 获取当前车辆状态时刻在上一周期轨迹中对应的位置点
  // (由于控制误差等因素，车辆当前时刻的位置可能与上一周期规划的本时刻位置是不一样的)
  // (能够运行到这里说明，当前时刻肯定在上一周期轨迹的起始时刻和终止时刻之间，也就说说按照time_match_index
  // 一定能够取到上一周期轨迹中与车辆当前时刻对应的位置点)
  auto time_matched_point =
      prev_trajectory->TrajectoryPointAt(time_matched_index);
  // 这一步是以防万一，正常情况下，按照上一步的描述，肯定能够取到上一周期轨迹中与车辆当前时刻对应的位置点
  if (!time_matched_point.has_path_point()) {
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }

  // 在空间上考虑
  
  // 获取车辆当前位置在上一周期轨迹中对应的轨迹点(即上一周期轨迹中与当前车辆位置最近的点)
  std::size_t position_matched_index = prev_trajectory->QueryNearestPoint(
      {vehicle_state.x(), vehicle_state.y()});

  // 求出当前时刻车辆位置的 frenet 坐标，求出的frenet坐标是相对于参考线上的位置匹配点的坐标
  auto frenet_sd = ComputePositionProjection(
      vehicle_state.x(), vehicle_state.y(),
      prev_trajectory->TrajectoryPointAt(position_matched_index));

  // 计算上一周期的时间匹配位置点(该点在refrence_line上)与车辆当前位置的横纵向偏差
  auto lon_diff = time_matched_point.path_point().s() - frenet_sd.first;
  auto lat_diff = frenet_sd.second;

  ADEBUG << "Control lateral diff: " << lat_diff
         << ", longitudinal diff: " << lon_diff;
  // 横纵向偏差过大,那么本周起规划从车辆当前状态开始
  if (std::fabs(lat_diff) > FLAGS_replan_lateral_distance_threshold ||
      std::fabs(lon_diff) > FLAGS_replan_longitudinal_distance_threshold) {
    AERROR << "the distance between matched point and actual position is too "
              "large. Replan is triggered. lat_diff = "
           << lat_diff << ", lon_diff = " << lon_diff;
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }
  // 从上一周期轨迹的时间匹配点再向后推一个规划周期(如果上一周期轨迹足够长,那么向后推的这一个周期对应的轨迹点,
  // 其实就是在上一个周期规划出的本周期时间段所对应的轨迹点)
  double forward_rel_time =
      prev_trajectory->TrajectoryPointAt(time_matched_index).relative_time() +
      planning_cycle_time;

  std::size_t forward_time_index =
      prev_trajectory->QueryLowerBoundPoint(forward_rel_time);

  ADEBUG << "Position matched index: " << position_matched_index;
  ADEBUG << "Time matched index: " << time_matched_index;
  // 为什么取最小值???,是为了保证车辆当前位置能够在截取的轨迹内
  // 注意到：如果取时间匹配点，那么说明当前时刻车辆走到l超出上一周期规划的本时刻的位置，说明车辆走“快”l；
  // 这个时候从时间匹配点开始截取，那么车辆当前位置一定在所截取的轨迹范围内。
  // 如果取位置匹配点，说明当前时刻车辆没有走到上一周期规划的本时刻的位置，说明车辆走“慢”了，这个时候虽
  // 然不能保证车辆当前位置一定在截取的轨迹范围内，但是所截取轨迹的第一个点是上一周期轨迹点中距离当前车辆位置
  // 最近的点。
  auto matched_index = std::min(time_matched_index, position_matched_index);
  
  // 从上一周期轨迹中,截取从match_index 到 forward_time_index的轨迹点
  std::vector<TrajectoryPoint> stitching_trajectory(
      prev_trajectory->trajectory_points().begin() +
          std::max(0, static_cast<int>(matched_index - 1)),
      prev_trajectory->trajectory_points().begin() + forward_time_index + 1);

  const double zero_s = time_matched_point.path_point().s();
  // 重新对截取的轨迹中的轨迹点赋予时间戳和累加距离s
  for (auto& tp : stitching_trajectory) {
    if (!tp.has_path_point()) {
      return ComputeReinitStitchingTrajectory(vehicle_state);
    }
	// tp.relative_time() + prev_trajectory->header_time()表示tp点的绝对时间
    tp.set_relative_time(tp.relative_time() + prev_trajectory->header_time() -
                         current_timestamp);
    tp.mutable_path_point()->set_s(tp.path_point().s() - zero_s);
  }
  *is_replan = false;
  // 返回截取并修改后的轨迹
  return stitching_trajectory;
}

std::pair<double, double> TrajectoryStitcher::ComputePositionProjection(
    const double x, const double y, const TrajectoryPoint& p) {
  Vec2d v(x - p.path_point().x(), y - p.path_point().y());
  Vec2d n(common::math::cos(
              common::math::Angle16::from_rad(p.path_point().theta())),
          common::math::sin(
              common::math::Angle16::from_rad(p.path_point().theta())));

  std::pair<double, double> frenet_sd;
  frenet_sd.first = v.InnerProd(n) + p.path_point().s();// 切向的坐标s
  frenet_sd.second = v.CrossProd(n);// 法向坐标l
  return frenet_sd;
}

}  // namespace planning
}  // namespace apollo

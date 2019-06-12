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

#include "modules/planning/tasks/path_decider/path_decider.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::VehicleConfigHelper;

PathDecider::PathDecider() : Task("PathDecider") {}

apollo::common::Status PathDecider::Execute(
    Frame *frame, ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(reference_line_info->path_data(),// 已经得到的路径规划的结果
                 reference_line_info->path_decision()); // 存储将要得到的路径决策结果
}

Status PathDecider::Process(const PathData &path_data,
                            PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  if (!MakeObjectDecision(path_data, path_decision)) {
    AERROR << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph decision ");
  }
  return Status::OK();
}

bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  const auto &frenet_points = frenet_path.points();
  if (frenet_points.empty()) {
    AERROR << "Path is empty.";
    return false;
  }
  // 半车宽
  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;

  const double lateral_stop_radius =
      half_width + FLAGS_static_decision_nudge_l_buffer;
  // 遍历每一个障碍物
  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto &obstacle = *path_obstacle->obstacle();
	// 判断是否为自行车或者行人
    bool is_bycycle_or_pedestrain =
        (obstacle.Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);
    // 1. 非自行车行人 且 非静态障碍物, 这里不考虑
    if (!is_bycycle_or_pedestrain && !obstacle.IsStatic()) {
      continue;
    }
    // 2. 前方和侧方已经存在忽略标签的障碍物，忽略
    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_ignore() &&
        path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_ignore()) {
      continue;
    }
	// 3. 障碍物存在前方停车标签，忽略
    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
	// 4. 障碍物存在侧方绕行标签，忽略
    if (path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_sidepass()) {
      // SIDE_PASS decision
      continue;
    }
    // 5. 障碍物所在区域在禁停区，忽略
    if (path_obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();
	
    // 6. 障碍物在规划路线以外，忽略
    if (sl_boundary.end_s() < frenet_points.front().s() ||
        sl_boundary.start_s() > frenet_points.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle.Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle.Id(),
                                        object_decision);
      continue;
    }
    // 7.如果障碍物和无人车侧方距离大于一个阈值(半车距离+3m)，那么障碍物侧方增加忽略标签
    // frenet_path上距离当前障碍物最近的那个点
    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
	// 说明障碍物的sl_boundary在frenet_point的左边或者右边,并且横向上有一段距离lateral_radius
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // ignore
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),
                                        object_decision);
    } 
	// 8. 否则如果障碍物和无人车侧方距离小于一个阈值(半车距离+0.5m)，那么就必须设置为停车
	else if (curr_l - lateral_stop_radius < sl_boundary.end_l() &&
               curr_l + lateral_stop_radius > sl_boundary.start_l()) {
      // stop
      *object_decision.mutable_stop() =
          GenerateObjectStopDecision(*path_obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle.Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle.Id(), object_decision);
      }
    } 
    // 9. 否者障碍物和无人车侧方距离在[半车距离+0.5m, 半车距离+3m]之间，允许左右微调，就将障碍物标签设置为微调
	else if (FLAGS_enable_nudge_decision) {
      // nudge
      if (curr_l - lateral_stop_radius > sl_boundary.end_l()) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle.Id(), object_decision);
      } else {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle.Id(), object_decision);
      }
    }
  }

  return true;
}

ObjectStop PathDecider::GenerateObjectStopDecision(
    const PathObstacle &path_obstacle) const {
  ObjectStop object_stop;

  double stop_distance = path_obstacle.MinRadiusStopDistance(
      VehicleConfigHelper::GetConfig().vehicle_param());
  object_stop.set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
  object_stop.set_distance_s(-stop_distance);

  const double stop_ref_s =
      path_obstacle.PerceptionSLBoundary().start_s() - stop_distance;
  const auto stop_ref_point =
      reference_line_info_->reference_line().GetReferencePoint(stop_ref_s);
  object_stop.mutable_stop_point()->set_x(stop_ref_point.x());
  object_stop.mutable_stop_point()->set_y(stop_ref_point.y());
  object_stop.set_stop_heading(stop_ref_point.heading());
  return object_stop;
}

}  // namespace planning
}  // namespace apollo

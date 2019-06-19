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

#include "modules/planning/tasks/dp_st_speed/dp_st_cost.h"

#include <algorithm>
#include <limits>

#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/speed/st_point.h"

namespace apollo {
namespace planning {
namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
}

DpStCost::DpStCost(const DpStSpeedConfig& config,
                   const std::vector<const PathObstacle*>& obstacles,
                   const common::TrajectoryPoint& init_point)
    : config_(config), obstacles_(obstacles), init_point_(init_point) {
  int index = 0;
  // 这里完成对boundary_map_这个集合的赋值。在boundary_map_中可以按照障碍物boundary的id()来搜索其对应的障碍物在obstacles
  // 中对应的序号
  for (auto& obstacle : obstacles) {
    boundary_map_[obstacle->st_boundary().id()] = index++;
  }
  
  unit_t_ = config_.total_time() / config_.matrix_dimension_t();

  AddToKeepClearRange(obstacles);
  // boundary_cost_ 是一个矩阵,每一行仍然是一个vector,每一行的元素是一个pair,这里全部初始化为(-1.0,-1.0)
  // 这个矩阵的一行代表一个障碍物,每一行中的每一个元素代表的是在时刻t的(s_upper, s_lower),每一行时间间隔是1,
  // 一行有8个元素,并且这个矩阵中行的排列顺序与obstacles向量中障碍物的排列顺序是一致的,也就是说这个矩阵的第一行
  // 对应于obstacles中的第一个障碍物
  boundary_cost_.resize(obstacles_.size());
  for (auto& vec : boundary_cost_) {
    vec.resize(config_.matrix_dimension_t(), std::make_pair(-1.0, -1.0));
  }
  accel_cost_.fill(-1.0);
  jerk_cost_.fill(-1.0);
}

void DpStCost::AddToKeepClearRange(
    const std::vector<const PathObstacle*>& obstacles) {
  for (const auto& obstacle : obstacles) {
    if (obstacle->st_boundary().IsEmpty()) {
      continue;
    }
    if (obstacle->st_boundary().boundary_type() !=
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    float start_s = obstacle->st_boundary().min_s();
    float end_s = obstacle->st_boundary().max_s();
    keep_clear_range_.emplace_back(start_s, end_s);
  }
  SortAndMergeRange(&keep_clear_range_);
}

void DpStCost::SortAndMergeRange(
    std::vector<std::pair<float, float>>* keep_clear_range) {
  if (!keep_clear_range || keep_clear_range->empty()) {
    return;
  }
  std::sort(keep_clear_range->begin(), keep_clear_range->end());
  std::size_t i = 0;
  std::size_t j = i + 1;
  while (j < keep_clear_range->size()) {
    if (keep_clear_range->at(i).second < keep_clear_range->at(j).first) {
      ++i;
      ++j;
    } else {
      keep_clear_range->at(i).second = std::max(keep_clear_range->at(i).second,
                                                keep_clear_range->at(j).second);
      ++j;
    }
  }
  keep_clear_range->resize(i + 1);
}

bool DpStCost::InKeepClearRange(float s) const {
  if (keep_clear_range_.empty()) {
    return false;
  }
  for (const auto& p : keep_clear_range_) {
    if (p.first <= s && p.second >= s) {
      return true;
    }
  }
  return false;
}

float DpStCost::GetObstacleCost(const StGraphPoint& st_graph_point) {
  const float s = st_graph_point.point().s(); // 取出要计算cost的元素中的s,t
  const float t = st_graph_point.point().t();

  float cost = 0.0;
  // 遍历每一个障碍物,考虑所有障碍物在st_graph_point点的cost
  for (const auto* obstacle : obstacles_) {
  	// 如果不是阻挡障碍物就不用管它
    if (!obstacle->IsBlockingObstacle()) {
      continue;
    }
    // 取出这个这个障碍物的标定框boundary
    auto boundary = obstacle->st_boundary();
    const float kIgnoreDistance = 200.0;
	// 如果boundary的底框很远,那么就不用管这个boundary
    if (boundary.min_s() > kIgnoreDistance) {
      continue;
    }
	// 如果当前这个元素的t小于boundary的min_t或者当前这个元素的t大于boundary的max_t,那么这个元素肯定在这个boundary的外面
	// 也就是说这元素对应的(s,t) 肯定不会和这个boundary相撞
    if (t < boundary.min_t() || t > boundary.max_t()) {
      continue;
    }
	// 如果这个障碍物是阻挡障碍物,并且(s,t)这个元素在boundary内部,那么(s,t)肯定和boundary相撞,于是就返回最大值kInf,表示
	// (s,t)这个元素的cost无穷大
    if (obstacle->IsBlockingObstacle() &&
        boundary.IsPointInBoundary(STPoint(s, t))) {
      return kInf;
    }
    double s_upper = 0.0;
    double s_lower = 0.0;
    // 获取boundary.id()的index标号
    int boundary_index = boundary_map_[boundary.id()];
	
	// boundary_cost_矩阵中的元素的初值都是(-1.0,-1.0)
	// 判断第boundary_index(于obstacles中障碍物的序号一致)个障碍物在index_t()时刻的元素(也就是s_upper,s_lower)是否已经赋值
	// 其实也就是判断一下第boundary_index个障碍物对应的标定框在第t时刻的上下界是否存储到了boundary_cost_矩阵中
    if (boundary_cost_[boundary_index][st_graph_point.index_t()].first < 0.0) {
      boundary.GetBoundarySRange(t, &s_upper, &s_lower);
      boundary_cost_[boundary_index][st_graph_point.index_t()] =
          std::make_pair(s_upper, s_lower);
    } else {
      s_upper = boundary_cost_[boundary_index][st_graph_point.index_t()].first;
      s_lower = boundary_cost_[boundary_index][st_graph_point.index_t()].second;
    }
	// 车辆在boundary的后面
    if (s < s_lower) {
      constexpr float kSafeTimeBuffer = 3.0;
      const float len = obstacle->obstacle()->Speed() * kSafeTimeBuffer;  // 计算安全距离
      if (s + len < s_lower) { // 条件满足不会撞上,就去进入下一次循环研究下一个障碍物
        continue;
      } // 会撞的情况下计算这个(s,t)元素的cost
	  else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((len - s_lower + s), 2);
      }
    }
	// 车辆在boundary的前面
	else if (s > s_upper) {
      const float kSafeDistance = 20.0;  // or calculated from velocity
      if (s > s_upper + kSafeDistance) {
        continue;
      } else {
        cost += config_.obstacle_weight() * config_.default_obstacle_cost() *
                std::pow((kSafeDistance + s_upper - s), 2);
      }
    }// 到这里就能求出当前障碍物在t时刻s位置时的障碍物cost
  }
  // 到这里就能求出所有障碍物在t时刻s位置时的障碍物cost
  return cost * unit_t_;
}

float DpStCost::GetReferenceCost(const STPoint& point,
                                 const STPoint& reference_point) const {
  return config_.reference_weight() * (point.s() - reference_point.s()) *
         (point.s() - reference_point.s()) * unit_t_;
}

float DpStCost::GetSpeedCost(const STPoint& first, const STPoint& second,
                             const float speed_limit) const {
  float cost = 0.0;
  const float speed = (second.s() - first.s()) / unit_t_;
  if (speed < 0) {
    return kInf;
  }

  if (speed < FLAGS_max_stop_speed && InKeepClearRange(second.s())) {
    // first.s in range
    cost += config_.keep_clear_low_speed_penalty() * unit_t_ *
            config_.default_speed_cost();
  }

  float det_speed = (speed - speed_limit) / speed_limit;
  if (det_speed > 0) {
    cost += config_.exceed_speed_penalty() * config_.default_speed_cost() *
            fabs(speed * speed) * unit_t_;
  } else if (det_speed < 0) {
    cost += config_.low_speed_penalty() * config_.default_speed_cost() *
            -det_speed * unit_t_;
  }
  return cost;
}

float DpStCost::GetAccelCost(const float accel) {
  float cost = 0.0;
  constexpr float kEpsilon = 0.1;
  constexpr size_t kShift = 100;
  const size_t accel_key = static_cast<size_t>(accel / kEpsilon + 0.5 + kShift);
  DCHECK_LT(accel_key, accel_cost_.size());
  if (accel_key >= accel_cost_.size()) {
    return kInf;
  }

  if (accel_cost_.at(accel_key) < 0.0) {
    const float accel_sq = accel * accel;
    float max_acc = config_.max_acceleration();
    float max_dec = config_.max_deceleration();
    float accel_penalty = config_.accel_penalty();
    float decel_penalty = config_.decel_penalty();

    if (accel > 0.0) {
      cost = accel_penalty * accel_sq;
    } else {
      cost = decel_penalty * accel_sq;
    }
    cost += accel_sq * decel_penalty * decel_penalty /
                (1 + std::exp(1.0 * (accel - max_dec))) +
            accel_sq * accel_penalty * accel_penalty /
                (1 + std::exp(-1.0 * (accel - max_acc)));
    accel_cost_.at(accel_key) = cost;
  } else {
    cost = accel_cost_.at(accel_key);
  }
  return cost * unit_t_;
}

float DpStCost::GetAccelCostByThreePoints(const STPoint& first,
                                          const STPoint& second,
                                          const STPoint& third) {
  float accel = (first.s() + third.s() - 2 * second.s()) / (unit_t_ * unit_t_);
  return GetAccelCost(accel);
}

float DpStCost::GetAccelCostByTwoPoints(const float pre_speed,
                                        const STPoint& pre_point,
                                        const STPoint& curr_point) {
  float current_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  float accel = (current_speed - pre_speed) / unit_t_;
  return GetAccelCost(accel);
}

float DpStCost::JerkCost(const float jerk) {
  float cost = 0.0;
  constexpr float kEpsilon = 0.1;
  constexpr size_t kShift = 200;
  const size_t jerk_key = static_cast<size_t>(jerk / kEpsilon + 0.5 + kShift);
  if (jerk_key >= jerk_cost_.size()) {
    return kInf;
  }

  if (jerk_cost_.at(jerk_key) < 0.0) {
    float jerk_sq = jerk * jerk;
    if (jerk > 0) {
      cost = config_.positive_jerk_coeff() * jerk_sq * unit_t_;
    } else {
      cost = config_.negative_jerk_coeff() * jerk_sq * unit_t_;
    }
    jerk_cost_.at(jerk_key) = cost;
  } else {
    cost = jerk_cost_.at(jerk_key);
  }

  // TODO(All): normalize to unit_t_
  return cost;
}

float DpStCost::GetJerkCostByFourPoints(const STPoint& first,
                                        const STPoint& second,
                                        const STPoint& third,
                                        const STPoint& fourth) {
  float jerk = (fourth.s() - 3 * third.s() + 3 * second.s() - first.s()) /
               (unit_t_ * unit_t_ * unit_t_);
  return JerkCost(jerk);
}

float DpStCost::GetJerkCostByTwoPoints(const float pre_speed,
                                       const float pre_acc,
                                       const STPoint& pre_point,
                                       const STPoint& curr_point) {
  const float curr_speed = (curr_point.s() - pre_point.s()) / unit_t_;
  const float curr_accel = (curr_speed - pre_speed) / unit_t_;
  const float jerk = (curr_accel - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

float DpStCost::GetJerkCostByThreePoints(const float first_speed,
                                         const STPoint& first,
                                         const STPoint& second,
                                         const STPoint& third) {
  const float pre_speed = (second.s() - first.s()) / unit_t_;
  const float pre_acc = (pre_speed - first_speed) / unit_t_;
  const float curr_speed = (third.s() - second.s()) / unit_t_;
  const float curr_acc = (curr_speed - pre_speed) / unit_t_;
  const float jerk = (curr_acc - pre_acc) / unit_t_;
  return JerkCost(jerk);
}

}  // namespace planning
}  // namespace apollo

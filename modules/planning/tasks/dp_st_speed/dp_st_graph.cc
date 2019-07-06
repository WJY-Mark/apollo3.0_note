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
 * @file dp_st_graph.cc
 **/

#include "modules/planning/tasks/dp_st_speed/dp_st_graph.h"

#include <algorithm>
#include <limits>
#include <string>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/log.h"
#include "modules/common/math/vec2d.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::SpeedPoint;
using apollo::common::Status;
using apollo::common::VehicleParam;
using apollo::common::math::Vec2d;

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();

bool CheckOverlapOnDpStGraph(const std::vector<const StBoundary*>& boundaries,
                             const StGraphPoint& p1, const StGraphPoint& p2) {
  // 以p1和p2构建一条线段
  const common::math::LineSegment2d seg(p1.point(), p2.point());
  // 遍历所有的障碍物boundary
  for (const auto* boundary : boundaries) {
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
	// 如果有某个boundary和起始点构造的线段有重叠,说明从起点直接到终点(1s的时间)会撞上障碍物
    if (boundary->HasOverlap(seg)) {
      return true;
    }
  }
  return false;
}
}  // namespace

DpStGraph::DpStGraph(const StGraphData& st_graph_data,
                     const DpStSpeedConfig& dp_config,
                     const std::vector<const PathObstacle*>& obstacles,
                     const common::TrajectoryPoint& init_point,
                     const SLBoundary& adc_sl_boundary)
    : st_graph_data_(st_graph_data),
      dp_st_speed_config_(dp_config),
      obstacles_(obstacles),
      init_point_(init_point),
      dp_st_cost_(dp_config, obstacles, init_point_),
      adc_sl_boundary_(adc_sl_boundary) {
  dp_st_speed_config_.set_total_path_length(
      std::fmin(dp_st_speed_config_.total_path_length(),
                st_graph_data_.path_data_length()));
  unit_s_ = dp_st_speed_config_.total_path_length() /       // total_path_length() = 149
            (dp_st_speed_config_.matrix_dimension_s() - 1); // matrix_dimension_s() = 150 // 矩阵的列数
  unit_t_ = dp_st_speed_config_.total_time() /              // total_time() = 7.0 
            (dp_st_speed_config_.matrix_dimension_t() - 1); // matrix_dimension_t() = 8  //矩阵的行数
}


// 这个函数的作用时在st图上搜索速度曲线
Status DpStGraph::Search(SpeedData* const speed_data) {
  constexpr float kBounadryEpsilon = 1e-2;
  // 遍历每个st框
  for (const auto& boundary : st_graph_data_.st_boundaries()) {
  	// 如果st框的类型为KEEP_CLEAR,那么就忽略这个框
    if (boundary->boundary_type() == StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }
	 // 如果坐标原点在boundary上或者(min_t和min_s都非常接近原点),说明在这种情况下,车辆基本上被这个boundary挡住了
    if (boundary->IsPointInBoundary({0.0, 0.0}) ||
        (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
         std::fabs(boundary->min_s()) < kBounadryEpsilon)) {
      std::vector<SpeedPoint> speed_profile;
      float t = 0.0;
	  // 所以,每一时刻的速度都赋值为0,步长为unit_t_,直接得到了速度规划的结果
	  // unit_t_ = 1, matrix_dimension_t() = 8
      for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t(); 
           ++i, t += unit_t_) {
        SpeedPoint speed_point;
        speed_point.set_s(0.0);
        speed_point.set_t(t);
        speed_profile.emplace_back(speed_point);
      }
      // 将速度规划的结果speed_profile赋值给speed_data(其实speed_profile并不是一条曲线,在时间域上是从0到7,但是每个s都是0)
      speed_data->set_speed_vector(speed_profile);
      return Status::OK();
    }
  }
  // 如果st_boundaries为空, 说明在本周期中没有得到障碍物的boundary,也就是没有障碍物影响到车辆的速度规划
  if (st_graph_data_.st_boundaries().empty()) {
    ADEBUG << "No path obstacles, dp_st_graph output default speed profile.";
    std::vector<SpeedPoint> speed_profile;
    float s = 0.0;
    float t = 0.0;
	// 这里其实只是将时间t[0,7],以及s[0,7]范围的点赋值。(0,0),(1,1),(2,2),(3,3),(4,4),(5,5),(6,6),(7,7)
    for (int i = 0; i < dp_st_speed_config_.matrix_dimension_t() &&
                    i < dp_st_speed_config_.matrix_dimension_s();
         ++i, t += unit_t_, s += unit_s_) {
      SpeedPoint speed_point;
      speed_point.set_s(s);
      speed_point.set_t(t);
      const float v_default = unit_s_ / unit_t_;// 默认速度,这里计算出来是1
      speed_point.set_v(v_default);
      speed_point.set_a(0.0);// 加速度为0
      speed_profile.emplace_back(std::move(speed_point));
    }
    // std::move是将左值(或左值引用)转换到右值(右值引用)。将对象的状态或者所有权从一个对象转移到另一个对象,
    // 只是转移，没有内存的搬迁或者内存拷贝。比如这里,是将speed_profile的内存的所有权转移到speed_data。
    speed_data->set_speed_vector(std::move(speed_profile));
    return Status::OK();
  }

 // 构建一个cost_table_表格,表格的行数是时间t,列数是累计距离s。也就是说,这里表格是8行150列。表格的每个格子中的元素是StGraphPoint
  if (!InitCostTable().ok()) {
    const std::string msg = "Initialize cost table failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!CalculateTotalCost().ok()) {
    const std::string msg = "Calculate total cost failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (!RetrieveSpeedProfile(speed_data).ok()) {
    const std::string msg = "Retrieve best speed profile failed.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  return Status::OK();
}

Status DpStGraph::InitCostTable() {
  uint32_t dim_s = dp_st_speed_config_.matrix_dimension_s();  //150
  uint32_t dim_t = dp_st_speed_config_.matrix_dimension_t();  //8
  DCHECK_GT(dim_s, 2);
  DCHECK_GT(dim_t, 2);
  // cost_table_是一个以vector形式存储的矩阵,其中同一行表示的是在相同的t下的不同s,这个矩阵8行150列,这里使用
  // StGraphPoint()来初始化矩阵的每一个元素
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dim_t, std::vector<StGraphPoint>(dim_s, StGraphPoint()));

  float curr_t = 0.0;
  // 这里给cost_table_中每个元素的s和t赋值
  // 遍历每一行
  for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_) {
  	// 取出cost_table_的第一行,这一行的时间t是相同的,
    auto& cost_table_i = cost_table_[i];
    float curr_s = 0.0;
    // 遍历取出的那一行的每一列 赋值每一个矩阵元素的 s,t
    for (uint32_t j = 0; j < cost_table_i.size(); ++j, curr_s += unit_s_) {
      cost_table_i[j].Init(i, j, STPoint(curr_s, curr_t));
    }
  }
  // 得到一个矩阵cost_table_
  return Status::OK();
}

Status DpStGraph::CalculateTotalCost() {
  // col and row are for STGraph
  // t corresponding to col
  // s corresponding to row // 这里这个注释对应的是st图的行和列,不是cost_table_,二者正好是个转置
  uint32_t next_highest_row = 0;
  uint32_t next_lowest_row = 0;
  // 遍历每一个t
  for (size_t c = 0; c < cost_table_.size(); ++c) {
    int highest_row = 0;
    int lowest_row = cost_table_.back().size() - 1;
    // 遍历cost_table_ 的每一列
    // 第一次进这个循环的时候next_highest_row = 0,next_lowest_row = 0,保证能够计算第0列,也就是t = 0的那一列
    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      if (FLAGS_enable_multi_thread_in_dp_st_graph) {
        PlanningThreadPool::instance()->Push(
            std::bind(&DpStGraph::CalculateCostAt, this, c, r));
      } else {
	  	// 计算在第c行(对应时间t),第r列(对应累计距离s)的元素的障碍物cost
        CalculateCostAt(c, r);
      }
    }
	// 到此处,计算完成第c列的每一个元素的cost,也就是相同时刻t,不同s的总cost
    if (FLAGS_enable_multi_thread_in_dp_st_graph) {
      PlanningThreadPool::instance()->Synchronize();
    }
    // 这部分计算的next_highest_row和next_lowest_row作用就是为了减少计算量。例如当前节点是(t,s)，那么根据当前最大规划速度v
    // 就可以计算unit_t时刻以后，无人车的累积距离最大为s+3，那么就可以简单地将下一层计算限制在[t+1,s]到[t+1,s+3]内。而不用
    // 计算t+1行所有节点和当前节点的连接，因为部分节点当前节点过远，在unit_t以后根本到不了，所以不需要计算。
    for (uint32_t r = next_lowest_row; r <= next_highest_row; ++r) {
      const auto& cost_cr = cost_table_[c][r];
      if (cost_cr.total_cost() < std::numeric_limits<float>::infinity()) {
        int h_r = 0;
        int l_r = 0;
        GetRowRange(cost_cr, &h_r, &l_r);
        highest_row = std::max(highest_row, h_r);
        lowest_row = std::min(lowest_row, l_r);
      }
    }
    next_highest_row = highest_row;
    next_lowest_row = lowest_row;
  }

  return Status::OK();
}

void DpStGraph::GetRowRange(const StGraphPoint& point, int* next_highest_row,
                            int* next_lowest_row) {
  float v0 = 0.0;
  if (!point.pre_point()) {
    v0 = init_point_.v();
  } else {
    v0 = (point.index_s() - point.pre_point()->index_s()) * unit_s_ / unit_t_;
  }

  const int max_s_size = cost_table_.back().size() - 1;

  const float speed_coeff = unit_t_ * unit_t_;

  const float delta_s_upper_bound =
      v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff;
  *next_highest_row =
      point.index_s() + static_cast<int>(delta_s_upper_bound / unit_s_);
  if (*next_highest_row >= max_s_size) {
    *next_highest_row = max_s_size;
  }

  const float delta_s_lower_bound = std::fmax(
      0.0, v0 * unit_t_ + vehicle_param_.max_deceleration() * speed_coeff);
  *next_lowest_row =
      point.index_s() + static_cast<int>(delta_s_lower_bound / unit_s_);
  if (*next_lowest_row > max_s_size) {
    *next_lowest_row = max_s_size;
  } else if (*next_lowest_row < 0) {
    *next_lowest_row = 0;
  }
}

void DpStGraph::CalculateCostAt(const uint32_t c, const uint32_t r) {
  // 注意:cost_table_中存储的是从[c=0,r=0]到当前[c,r]的总cost
  auto& cost_cr = cost_table_[c][r];
  // 计算得到cost_table_[c][r] 的 障碍物cost,在计算时需要考虑cost_cr对应时刻的所有障碍物
  cost_cr.SetObstacleCost(dp_st_cost_.GetObstacleCost(cost_cr)); 
  // cost无穷大,直接返回(说明cost_cr对应的s直接落在的某个障碍物的boundary内部)
  if (cost_cr.obstacle_cost() > std::numeric_limits<float>::max()) {
    return;
  }

  const auto& cost_init = cost_table_[0][0];
  // 如果是第0列,也就是cost_table中时间t = 0 的那一列,
  if (c == 0) {
    DCHECK_EQ(r, 0) << "Incorrect. Row should be 0 with col = 0. row: " << r;
    cost_cr.SetTotalCost(0.0);
    return;
  }
  
  // 获取在unit_s_ * r处的速度限制,unit_s_ = 1
  float speed_limit =
      st_graph_data_.speed_limit().GetSpeedLimitByS(unit_s_ * r);

  // 如果是第1行,也就是t = 1的 那一行
  if (c == 1) {
  	// 求取第1行,第r列,也就是t = 1, s = r*unit_s_处的加速度,这里的加速速度是指从第0时刻的s=0位置直接到达函数输入中[c=1,r]对应
  	// 的位置时的加速度
    const float acc = (r * unit_s_ / unit_t_ - init_point_.v()) / unit_t_;
	// 如果加速度超出阈值,说明从[c=0,r=0]直接到达(1s的时间)这个点[c=1,r]是不可能的,直接返回这个时候的总cost只包含了障碍物cost
    if (acc < dp_st_speed_config_.max_deceleration() ||
        acc > dp_st_speed_config_.max_acceleration()) {
      return;
    }
    // 如果没有超出阈值,就要计算从[c=0,r=0]直接到达(1s的时间)这个点[c=1,r]cost_cr,在加速度上是可行的,但是要判断直接到达的话会不会
    // 撞上障碍物。如果会装上障碍物直接返回,这个时候的总cost只包含了障碍物cost
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                cost_init)) {
      return;
    }
	// 如果撞不上,那么[c=1,r]总的cost就是其障碍物cost + 初始点[c=0,r=0]的总cost + r对应s处的speed_cost,acc_cost,jerk_cost
    cost_cr.SetTotalCost(cost_cr.obstacle_cost() + cost_init.total_cost() +
                         CalculateEdgeCostForSecondCol(r, speed_limit));
	// 并且,[c=1,r]的上一个点就是初始点[0,0],说明可以直接从[0,0]直接在1s时间内到达[c=1,r]点,而不必逐个格子去走
    cost_cr.SetPrePoint(cost_init);
    return;
  }

  constexpr float kSpeedRangeBuffer = 0.20;
  // 这里计算的是在一个单位时间unit_t_=1s内最多能够走的距离包含多少个单位距离unit_s_
  const uint32_t max_s_diff =
      static_cast<uint32_t>(FLAGS_planning_upper_speed_limit *
                            (1 + kSpeedRangeBuffer) * unit_t_ / unit_s_);
  // 如果max_s_diff < r,那么说明在一个单位时间unit_t_=1s内,不可能从[c,r=0]直接到到达r对应的s,要想到达r对应的s至少得以r-max_s_diff
  // 作为起点才可以。如果max_s_diff > r,那么说明在一个单位时间unit_t_=1s内,能够从[c,r=0]直接到到达r对应的s
  // 所以这里求得的是在一个单位时间unit_t_=1s内,能够到达当前的s的那个最远的s所对应的r(r_low)
  const uint32_t r_low = (max_s_diff < r ? r - max_s_diff : 0);

  // 取出上一个时间([c-1]对应的时刻)的那一列赋给pre_col,注意pre_col中的每一个元素[c-1,r]的cost都已经计算好了
  const auto& pre_col = cost_table_[c - 1];
  
  // c = 2, 对应于t = 2那一列
  if (c == 2) {
  	// 从最远的能够到达当前r的那一层开始遍历,一直遍历到当前r-1对应的s
    for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
      const float acc =
          (r * unit_s_ - 2 * r_pre * unit_s_) / (unit_t_ * unit_t_);
	  // 这个地方为什么要乘个2这是因为平均加速度 = {(s2-s1)/t - (s1-s0)/t}/t,且s0 = 0
	  
	  // 如果计算出的加速度不在阈值范围之内,那么就考虑下一个s(r_pre)
      if (acc < dp_st_speed_config_.max_deceleration() ||
          acc > dp_st_speed_config_.max_acceleration()) {
        continue;
      }
      // 如果加速度在阈值范围内,就考虑从上一列中的r_pre这个点直接连接到当前这个点[c=2,r_pre]会不会和障碍物相撞
      // 如果相撞就考虑下一个s(r_pre)
      if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                  pre_col[r_pre])) {
        continue;
      }
      // 如果不相撞,cost = 当前点cost_cr[c,r]的障碍物cost + 上一时刻与cost_cr直连的点的总cost + 对应的speed_cost,acc_cost,jerk_cost
      const float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                         CalculateEdgeCostForThirdCol(r, r_pre, speed_limit);
      // 上面的处理是对于某个特定的s(和r_pre对应)来进行的,下面这个if就是为了在这些s中找到cost最小的那一个
      if (cost < cost_cr.total_cost()) {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPrePoint(pre_col[r_pre]);
      }
    }
    return;
  }
  for (uint32_t r_pre = r_low; r_pre <= r; ++r_pre) {
  	// 如果上一列(时间域)中对应的r_pre位置的cost为无穷大,那么说明,速度曲线根本就可能经过那个点,所以就不考虑了
  	// 如果上一列(时间域)中对应的r_pre位置的点没有与之相连的前向点pre_point(),说明速度曲线根本就可能经过那个点,所以就不考虑了
    if (std::isinf(pre_col[r_pre].total_cost()) ||
        pre_col[r_pre].pre_point() == nullptr) {
      continue;
    }

    const float curr_a = (cost_cr.index_s() * unit_s_ +
                          pre_col[r_pre].pre_point()->index_s() * unit_s_ -
                          2 * pre_col[r_pre].index_s() * unit_s_) /
                         (unit_t_ * unit_t_); // 平均加速度 = {(s2-s1)/t - (s1-s0)/t}/t
    // 如果加速度超出阈值范围,那么这个r_pre对应的s不再考虑,考虑下一个
    if (curr_a > vehicle_param_.max_acceleration() ||
        curr_a < vehicle_param_.max_deceleration()) {
      continue;
    }
	// 如果加速度在阈值范围之内,考虑从上一列(上一个时刻t)对应的r_pre出直连[c,r]点的线是否与障碍物的boundary有重叠
	// 如果有重叠,那么这个r_pre对应的s不再考虑,考虑下一个
    if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
                                pre_col[r_pre])) {
      continue;
    }
    // 如果没有重叠,就考察上上个点pre_col[r_pre].pre_point()的总cost
    uint32_t r_prepre = pre_col[r_pre].pre_point()->index_s();
    const StGraphPoint& prepre_graph_point = cost_table_[c - 2][r_prepre];
	// 如果上上个点的总cost为无穷大,那么这个r_pre对应的s不再考虑,考虑下一个
    if (std::isinf(prepre_graph_point.total_cost())) {
      continue;
    }
    // 如果上上个点没有pre_point(),那么这个r_pre对应的s不再考虑,考虑下一个
    if (!prepre_graph_point.pre_point()) {
      continue;
    }
	// 如果上上个点有pre_point(),那就取出上上上个点,上上个点,上个点,当前点来计算cost(这里的上指的是时间域)
    const STPoint& triple_pre_point = prepre_graph_point.pre_point()->point();
    const STPoint& prepre_point = prepre_graph_point.point();
    const STPoint& pre_point = pre_col[r_pre].point();
    const STPoint& curr_point = cost_cr.point();
    float cost = cost_cr.obstacle_cost() + pre_col[r_pre].total_cost() +
                 CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                   curr_point, speed_limit);

    if (cost < cost_cr.total_cost()) {
      cost_cr.SetTotalCost(cost);
      cost_cr.SetPrePoint(pre_col[r_pre]);
    }
  }
}

Status DpStGraph::RetrieveSpeedProfile(SpeedData* const speed_data) {
  float min_cost = std::numeric_limits<float>::infinity();
  const StGraphPoint* best_end_point = nullptr;
  // 遍历cost_table_中的最后一行,其实就是在t=7那一行中找出cost最小的那个点赋值给best_end_point,这个点能够保证时间是从0-7s,但是
  // 最终距离不一定是s = 149.
  for (const StGraphPoint& cur_point : cost_table_.back()) {
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }
  // 遍历cost_table_ 的每一行,取出每一行的最后一个元素,这个元素的s一定是149,然后从这里面找出cost最小的赋值给best_end_point,这个点
  // 一定能够到达s=149,但是不一定要花8秒的时间
  ////需要注意的是,最终选定的终点是这两种方式中cost最小的那一种,也就是说,最后选定的这个终点,可能是能够保证s但不保证t的,也可能是能
  ////够保证t但是不能够保证s的。
  for (const auto& row : cost_table_) {
    const StGraphPoint& cur_point = row.back();
    if (!std::isinf(cur_point.total_cost()) &&
        cur_point.total_cost() < min_cost) {
      best_end_point = &cur_point;
      min_cost = cur_point.total_cost();
    }
  }

  if (best_end_point == nullptr) {
    const std::string msg = "Fail to find the best feasible trajectory.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  std::vector<SpeedPoint> speed_profile;
  const StGraphPoint* cur_point = best_end_point;
  // 向前回溯得到速度点
  while (cur_point != nullptr) {
    SpeedPoint speed_point;
    speed_point.set_s(cur_point->point().s());
    speed_point.set_t(cur_point->point().t());
    speed_profile.emplace_back(speed_point);
    cur_point = cur_point->pre_point();
  }
  // 由于回溯时,最后一个速度点在列表的头部,所以这里要将速度曲线列表反转
  std::reverse(speed_profile.begin(), speed_profile.end());

  constexpr float kEpsilon = std::numeric_limits<float>::epsilon();
  if (speed_profile.front().t() > kEpsilon ||
      speed_profile.front().s() > kEpsilon) {
    const std::string msg = "Fail to retrieve speed profile.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  speed_data->set_speed_vector(speed_profile);
  return Status::OK();
}

float DpStGraph::CalculateEdgeCost(const STPoint& first, const STPoint& second,
                                   const STPoint& third, const STPoint& forth,
                                   const float speed_limit) {
  return dp_st_cost_.GetSpeedCost(third, forth, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(second, third, forth) +
         dp_st_cost_.GetJerkCostByFourPoints(first, second, third, forth);
}

float DpStGraph::CalculateEdgeCostForSecondCol(const uint32_t row,
                                               const float speed_limit) {
  float init_speed = init_point_.v();
  float init_acc = init_point_.a();
  const STPoint& pre_point = cost_table_[0][0].point();
  const STPoint& curr_point = cost_table_[1][row].point();
  return dp_st_cost_.GetSpeedCost(pre_point, curr_point, speed_limit) +
         dp_st_cost_.GetAccelCostByTwoPoints(init_speed, pre_point,
                                             curr_point) +
         dp_st_cost_.GetJerkCostByTwoPoints(init_speed, init_acc, pre_point,
                                            curr_point);
}

float DpStGraph::CalculateEdgeCostForThirdCol(const uint32_t curr_row,
                                              const uint32_t pre_row,
                                              const float speed_limit) {
  float init_speed = init_point_.v();
  const STPoint& first = cost_table_[0][0].point();
  const STPoint& second = cost_table_[1][pre_row].point();
  const STPoint& third = cost_table_[2][curr_row].point();
  return dp_st_cost_.GetSpeedCost(second, third, speed_limit) +
         dp_st_cost_.GetAccelCostByThreePoints(first, second, third) +
         dp_st_cost_.GetJerkCostByThreePoints(init_speed, first, second, third);
}

}  // namespace planning
}  // namespace apollo

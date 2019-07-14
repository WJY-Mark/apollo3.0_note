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

#include "modules/planning/tasks/st_graph/st_boundary_mapper.h"

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/file.h"
#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::PathPoint;
using apollo::common::SLPoint;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleParam;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;
using apollo::common::util::StrCat;

namespace {
constexpr double boundary_t_buffer = 0.1;
constexpr double boundary_s_buffer = 1.0;
}  // namespace

StBoundaryMapper::StBoundaryMapper(const SLBoundary& adc_sl_boundary,
                                   const StBoundaryConfig& config,
                                   const ReferenceLine& reference_line,
                                   const PathData& path_data,
                                   const double planning_distance,
                                   const double planning_time,
                                   bool is_change_lane)
    : adc_sl_boundary_(adc_sl_boundary),
      st_boundary_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()),
      planning_distance_(planning_distance),
      planning_time_(planning_time),
      is_change_lane_(is_change_lane) {}

Status StBoundaryMapper::CreateStBoundary(PathDecision* path_decision) const {
  const auto& path_obstacles = path_decision->path_obstacles();
  // planning_time_ = 7.0s
  if (planning_time_ < 0.0) {
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }
  // 期望路径的离散路径点太少的话,直接返回规划错误状态
  if (path_data_.discretized_path().NumOfPoints() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().NumOfPoints() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();// min_stop_s被赋值为double类型的最大值
  // 遍历路径上的每一个障碍物
  for (const auto* const_path_obstacle : path_obstacles.Items()) {// item = {id,obstacle}
  	// 按照ID查询出对应的障碍物path_obstacle
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());
	// 如果障碍物没有纵向决策标签, 那么对该障碍物与期望路径做碰撞分析
    if (!path_obstacle->HasLongitudinalDecision()) {
	  // 对于某个没有纵向决策标签的障碍物如果能够在st图上成功构建它的的st边界框,返回ok,{}里面不执行,
      if (!MapWithoutDecision(path_obstacle).ok()) {
        std::string msg = StrCat("Fail to map obstacle ", path_obstacle->Id(),
                                 " without decision.");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
	  
      continue;// 而是继续构建没有纵向决策标签障碍物的st边界框
    }// 到此处,所有没有纵向决策标签的障碍物全部都已经构建了st边界框

	// 上面是处理的没有纵向决策标签的障碍物,下面处理有纵向决策标签的障碍物

	// 获取障碍物的纵向决策标签
    const auto& decision = path_obstacle->LongitudinalDecision();
	// 对于决策为停车的障碍物
    if (decision.has_stop()) {
		// 求取停车点的累计距离stop_s
      const double stop_s = path_obstacle->PerceptionSLBoundary().start_s() +
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 1.0;
	  // 如果stop_s 比车辆的后边框还靠后,说明这里这里有问题,返回错误状态
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {
        AERROR << "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_ERROR, "invalid decision");
      }
	  // 这一步处理是为了在所有的有纵向决策标签的障碍物中找到最小的停车距离,并得到停车障碍物stop_obstacle
      if (stop_s < min_stop_s) {
        stop_obstacle = path_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } 
	// 如果纵向决策标签是跟随,超车或者让行
	else if (decision.has_follow() || decision.has_overtake() ||decision.has_yield()) 
   {  // 生成对应的st边界框,并标定边界框的类型为跟随,超车,让行
      if (!MapWithDecision(path_obstacle, decision).ok()) {
        AERROR << "Fail to map obstacle " << path_obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } 
	else {
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }
  // 如果有需要停车的障碍物
  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

Status StBoundaryMapper::CreateStBoundaryWithHistory(
    const ObjectDecisions& history_decisions,
    PathDecision* path_decision) const {
  const auto& path_obstacles = path_decision->path_obstacles();
  if (planning_time_ < 0.0) {
    const std::string msg = "Fail to get params since planning_time_ < 0.";
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  if (path_data_.discretized_path().NumOfPoints() < 2) {
    AERROR << "Fail to get params because of too few path points. path points "
              "size: "
           << path_data_.discretized_path().NumOfPoints() << ".";
    return Status(ErrorCode::PLANNING_ERROR,
                  "Fail to get params because of too few path points");
  }

  std::unordered_map<std::string, ObjectDecisionType> prev_decision_map;
  for (const auto& history_decision : history_decisions.decision()) {
    for (const auto& decision : history_decision.object_decision()) {
      if (PathObstacle::IsLongitudinalDecision(decision) &&
          !decision.has_ignore()) {
        prev_decision_map[history_decision.id()] = decision;
        break;
      }
    }
  }

  PathObstacle* stop_obstacle = nullptr;
  ObjectDecisionType stop_decision;
  double min_stop_s = std::numeric_limits<double>::max();

  for (const auto* const_path_obstacle : path_obstacles.Items()) {
    auto* path_obstacle = path_decision->Find(const_path_obstacle->Id());
    auto iter = prev_decision_map.find(path_obstacle->Id());
    ObjectDecisionType decision;
    if (iter == prev_decision_map.end()) {
      decision.mutable_ignore();
    } else {
      decision = iter->second;
    }

    if (!path_obstacle->HasLongitudinalDecision()) {
      if (!MapWithoutDecision(path_obstacle).ok()) {
        std::string msg = StrCat("Fail to map obstacle ", path_obstacle->Id(),
                                 " without decision.");
        AERROR << msg;
        return Status(ErrorCode::PLANNING_ERROR, msg);
      }
      if (path_obstacle->st_boundary().IsEmpty() || decision.has_ignore()) {
        continue;
      }
    }
    if (path_obstacle->HasLongitudinalDecision()) {
      decision = path_obstacle->LongitudinalDecision();
    }
    if (decision.has_stop()) {
      const double stop_s = path_obstacle->PerceptionSLBoundary().start_s() +
                            decision.stop().distance_s();
      // this is a rough estimation based on reference line s, so that a large
      // buffer is used.
      constexpr double stop_buff = 1.0;
      if (stop_s + stop_buff < adc_sl_boundary_.end_s()) {
        AERROR << "Invalid stop decision. not stop at behind of current "
                  "position. stop_s : "
               << stop_s << ", and current adc_s is; "
               << adc_sl_boundary_.end_s();
        return Status(ErrorCode::PLANNING_ERROR, "invalid decision");
      }
      if (stop_s < min_stop_s) {
        stop_obstacle = path_obstacle;
        min_stop_s = stop_s;
        stop_decision = decision;
      }
    } else if (decision.has_follow() || decision.has_overtake() ||
               decision.has_yield()) {
      if (!MapWithDecision(path_obstacle, decision).ok()) {
        AERROR << "Fail to map obstacle " << path_obstacle->Id()
               << " with decision: " << decision.DebugString();
        return Status(ErrorCode::PLANNING_ERROR,
                      "Fail to map overtake/yield decision");
      }
    } else {
      AWARN << "No mapping for decision: " << decision.DebugString();
    }
  }

  if (stop_obstacle) {
    bool success = MapStopDecision(stop_obstacle, stop_decision);
    if (!success) {
      std::string msg = "Fail to MapStopDecision.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
  return Status::OK();
}

bool StBoundaryMapper::MapStopDecision(
    PathObstacle* stop_obstacle,
    const ObjectDecisionType& stop_decision) const {
  DCHECK(stop_decision.has_stop()) << "Must have stop decision";
  // 停车障碍物boundary的start_s在本周期规划路径总长度之外,那么本周期中不用
  if (stop_obstacle->PerceptionSLBoundary().start_s() >
      adc_sl_boundary_.end_s() + planning_distance_) {
    return true;
  }

  double st_stop_s = 0.0;
  // 相对于车辆后轴中心的停车距离点stop_ref_s
  const double stop_ref_s = stop_obstacle->PerceptionSLBoundary().start_s() +
                            stop_decision.stop().distance_s() -
                            vehicle_param_.front_edge_to_center();
  // 如果stop_ref_s在本周期规划路径的最后一个路点之外,那么求取在离散点路径上的停车st_stop_s
  // frenet_frame_path是以车辆后轴中心为原点的路径
  if (stop_ref_s > path_data_.frenet_frame_path().points().back().s()) {
    st_stop_s =
        path_data_.discretized_path().EndPoint().s() +
        (stop_ref_s - path_data_.frenet_frame_path().points().back().s());
  } 
  // 这里对应的是stop_ref_s在本周期规划路径范围之内的情况
  else {
    PathPoint stop_point;
	// 
    if (!path_data_.GetPathPointWithRefS(stop_ref_s, &stop_point)) {
      AERROR << "Fail to get path point from reference s. The sl boundary of "
                "stop obstacle "
             << stop_obstacle->Id()
             << " is: " << stop_obstacle->PerceptionSLBoundary().DebugString();
      return false;
    }
    // 得到车辆在期望离散路径path_data_.discretized_path()上的停车点st_stop_s
    st_stop_s = stop_point.s();
  }

  constexpr double kStopEpsilon = 1e-2;
  // 停车下边界
  const double s_min = std::max(0.0, st_stop_s - kStopEpsilon);
  // 停车上边界：当st_stop_s在本周期规划路径范围之内时,上边界就是期望路径的长度或者指引线长度(因为超出这个范围就没路了)
  // 当st_stop_s在本周期规划路径范围之外时,上边界有可能时s_min或者期望路径长度或者指引线长度
  const double s_max =
      std::fmax(s_min, std::fmax(planning_distance_, reference_line_.Length()));

  std::vector<std::pair<STPoint, STPoint>> point_pairs;
  
  // 在本周期0时刻时,停车的下边界和上边界
  point_pairs.emplace_back(STPoint(s_min, 0.0), STPoint(s_max, 0.0));
  // 在本周期规划的终止时刻,停车的上下界
  point_pairs.emplace_back(
      STPoint(s_min, planning_time_),
      STPoint(s_max + st_boundary_config_.boundary_buffer(), planning_time_));

  // 构造停车障碍物的stboundary
  auto boundary = StBoundary(point_pairs);
  // 这个boundary类型为STOP
  boundary.SetBoundaryType(StBoundary::BoundaryType::STOP);
  boundary.SetCharacteristicLength(st_boundary_config_.boundary_buffer());
  boundary.SetId(stop_obstacle->Id());
  stop_obstacle->SetStBoundary(boundary);
  return true;
}

Status StBoundaryMapper::MapWithoutDecision(PathObstacle* path_obstacle) const {
  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  // 传入的参数:期望路径点,路径上的障碍物,障碍物轨迹预测点标定框的上下界(是这个函数执行的结果)。如果成功找出了
  // 该障碍物的上下界标定框就返回true
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();
  }
  // 如果这个障碍物的标定框上下界成功找到,就会往下执行
  // 转成StBoundary，存入PathObstacle中
  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);
  // boundary的id与障碍物的id是一致的
  boundary.SetId(path_obstacle->Id());
  const auto& prev_st_boundary = path_obstacle->st_boundary();
  const auto& ref_line_st_boundary =
      path_obstacle->reference_line_st_boundary();
  if (!prev_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(prev_st_boundary.boundary_type());
  } else if (!ref_line_st_boundary.IsEmpty()) {
    boundary.SetBoundaryType(ref_line_st_boundary.boundary_type());
  }
  // 到此处在st图上构建了这个障碍物的st 边界框
  path_obstacle->SetStBoundary(boundary);
  return Status::OK();
}

// 这个函数的作用是,根据输入的障碍物和期望路径,分析该障碍物预测轨迹的每一个点与期望路径的每一个点做碰撞分析,
// 然后得到与期望路径有碰撞的每一个轨迹预测点的标定框的上下界
bool StBoundaryMapper::GetOverlapBoundaryPoints(
    const std::vector<PathPoint>& path_points, const Obstacle& obstacle,
    std::vector<STPoint>* upper_points,
    std::vector<STPoint>* lower_points) const {
  DCHECK_NOTNULL(upper_points);
  DCHECK_NOTNULL(lower_points);
  DCHECK(upper_points->empty());
  DCHECK(lower_points->empty());
  DCHECK_GT(path_points.size(), 0);

  if (path_points.empty()) {
    AERROR << "No points in path_data_.discretized_path().";
    return false;
  }
  // 获取当前障碍物的预测轨迹点
  const auto& trajectory = obstacle.Trajectory();
  // 如果障碍物预测轨迹的size为0,说明该障碍物没有预测轨迹,那么只考虑它本身所在的位置与车辆是否相撞即可
  // 这个时候这个障碍物的标定框粗略的求取就可以
  if (trajectory.trajectory_point_size() == 0) {
    if (!obstacle.IsStatic()) {
      AWARN << "Non-static obstacle[" << obstacle.Id()
            << "] has NO prediction trajectory."
            << obstacle.Perception().ShortDebugString();
    }
	// 遍历期望路径的每一个路径点,找到期望路径上第一个与障碍物碰撞的路点,并根据这个路点求取该障碍物的边框上下界
    for (const auto& curr_point_on_path : path_points) {
	  // 如果当前期望路径点的累计长度s比期望路径的长度还大,说明这个路径点已经超出了当前周期所关注期望路径的范围,
	  // 就不用再考虑了, 也就是说这个时候本周期的期望路径碰撞分析已经完毕。(在计算期望路径的时候我们可以计算到很
	  // 远的距离,但是在实际使用中,总是将其限定在某一长度之内,因为未来时刻环境会变化,当前周期计算得到的很远的路
	  // 径点在未来时刻可能就不对了)
      if (curr_point_on_path.s() > planning_distance_) {
        break;
      }
	  // 获取这个障碍物的box
      const Box2d obs_box = obstacle.PerceptionBoundingBox();
      // 如果当前轨迹预测点curr_point_on_path的box(其实是车辆后轴中心在这个路径点时的box) 与障碍物的box有重叠
      // 如果有重叠,说明该障碍物与当前路径点curr_point_on_path有碰撞,就要计算障碍物这个位置的标定框上下界
      if (CheckOverlap(curr_point_on_path, obs_box,
                       st_boundary_config_.boundary_buffer())) {
        const double backward_distance = -vehicle_param_.front_edge_to_center();
        const double forward_distance = vehicle_param_.length() +
                                        vehicle_param_.width() +
                                        obs_box.length() + obs_box.width();
        // 不发生碰撞有两种情况:
        // 1. 在当前时刻,车辆没有到达障碍物,对应车辆能至远够到达的位置极限位low_s,也就是障碍物标定框的下界low_s。					
        // 车辆的后轴中心到达等于low_s的点时,车头恰好顶在障碍物上,所以low_s是这个障碍物的下边界也就是说车辆
        // 如果走到curr_point_on_path就会和这个障碍物相撞,车辆最多走到low_s的情况下恰好和障碍物
        // 不撞,所以low_s就是这个障碍物运动的下界,关于low_s怎么求取看下面
		double low_s =
            std::fmax(0.0, curr_point_on_path.s() + backward_distance);
		// 2. 车辆已经超越了障碍物,对应车辆至少到达的位置位high_s,也就是障碍物的标定框上界
        double high_s = std::fmin(planning_distance_,
                                  curr_point_on_path.s() + forward_distance);
        lower_points->emplace_back(low_s, 0.0);
        lower_points->emplace_back(low_s, planning_time_);
        upper_points->emplace_back(high_s, 0.0);
        upper_points->emplace_back(high_s, planning_time_);
        break;
      }
    }
  } 
  // else中处理障碍物有预测轨迹点的情况
  else {
    const int default_num_point = 50;
    DiscretizedPath discretized_path;
	// 如果期望路径path_points中路径点的个数超过100个,那么对其重新采样,比如:path_points原来有180个点,那么
	// 现在就每隔3(3.6)个点采样一个点,就会得到60个离散采样点,这么做的目的是为了降低采样的次数,减小计算量
    if (path_points.size() > 2 * default_num_point) {
      const int ratio = path_points.size() / default_num_point;
      std::vector<PathPoint> sampled_path_points;
	  // 重新采样得到的点存入到sampled_path_points列表中
      for (size_t i = 0; i < path_points.size(); ++i) {
        if (i % ratio == 0) {
          sampled_path_points.push_back(path_points[i]);
        }
      }
      discretized_path.set_path_points(sampled_path_points);
    } 
	else {
      discretized_path.set_path_points(path_points);
    }
	// 遍历障碍物预测轨迹点的每一个点,为每一个障碍物预测轨迹点求取与期望路径上第一个碰撞的路点,进而根据这个路径点
	// 来求取与之碰撞的障碍物预测轨迹点的标定框
    for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
	  // 取出障碍物预测轨迹点的第i个轨迹点
      const auto& trajectory_point = trajectory.trajectory_point(i);
	  
	  // 构建障碍物在轨迹点trajectory_point(i)上的box
      const Box2d obs_box = obstacle.GetBoundingBox(trajectory_point);
      // 获取障碍物在轨迹点trajectory_point(i)的相对时间,相对于本周期规划起始点init_point的时间
      double trajectory_point_time = trajectory_point.relative_time();
      constexpr double kNegtiveTimeThreshold = -1.0;
	  // 条件满足,说明这个障碍物轨迹点是在本周期起始轨迹点时刻1s以前该障碍物所在的位置,那么这个预测轨迹点就不用考虑了
      if (trajectory_point_time < kNegtiveTimeThreshold) {
        continue;
      }

      const double step_length = vehicle_param_.front_edge_to_center();// 车头到后轴中心的距离
      // 遍历重新采样的离散路径discretized_path的所有路点,找出和某个时刻障碍物obs_box有交叠的第一个路点,我们关注的
      // 就是第一个发生碰撞的点,基于这个点求取当前障碍物轨迹预测点的标定框上下界,遍历路径时的采样步长为step_length
      for (double path_s = 0.0; path_s < discretized_path.Length();
           path_s += step_length) {
		// 从期望路径上求取累计长度为path_s的对应路点
        const auto curr_adc_path_point = discretized_path.Evaluate(
            path_s + discretized_path.StartPoint().s());
		// 判断curr_adc_path_point与障碍物轨迹预测点处的box是否重叠
        if (CheckOverlap(curr_adc_path_point, obs_box,
                         st_boundary_config_.boundary_buffer())) {
          // found overlap, start searching with higher resolution
          // 因为这个路点是第一个与当前时刻障碍物有交叠的路点,所以上一个路点一定与障碍物没有交叠,而且路点采样的
          // 步长时step_length,所以从第一个与障碍物有交叠的路点减去step_length对应的那个s一定与障碍物没有交叠,所
          // 以就以此作为当前时刻障碍物标定框的粗略下界
          const double backward_distance = -step_length;
		  // 对应于最极限的情况,沿着路径的方向,车辆右前角撞到了障碍物的左后角,那么车辆所在路点加上车辆和障碍物的
		  // box的对角线长度就是车辆恰好超越障碍物不发生碰撞,所以这里这样设置forward_distance肯定能够保证车辆超越
		  // 障碍物不发生碰撞
          const double forward_distance = vehicle_param_.length() +
                                          vehicle_param_.width() +
                                          obs_box.length() + obs_box.width();
          const double default_min_step = 0.1;  // in meters
          
          const double fine_tuning_step_length = std::fmin(
              default_min_step, discretized_path.Length() / default_num_point);

          bool find_low = false;
          bool find_high = false;
          double low_s = std::fmax(0.0, path_s + backward_distance);
		  // 如果车辆box和障碍物box重叠,那么车辆所在路点path_s加上forward_distance 肯定能够超越障碍物,
		  // 绝对不会碰撞,但是加上forward_distance有可能加的大多了,也就是说在一些情况下,不用加上forward_distance
		  // 这么大的数,就能够保证车辆与障碍物的box不重叠
          double high_s =
              std::fmin(discretized_path.Length(), path_s + forward_distance);
          // 所以在这里采用步步紧逼的方法逐步降低high_s,增大low_s,从而得到一个较小的又不会碰撞的障碍物标定框，
          // 给速度规划留下更大的空间
          while (low_s < high_s) {
            if (find_low && find_high) {
              break;
            }
            if (!find_low) {
              const auto& point_low = discretized_path.Evaluate(
                  low_s + discretized_path.StartPoint().s());
              if (!CheckOverlap(point_low, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                low_s += fine_tuning_step_length;
              } else {
                find_low = true;
              }
            }
            if (!find_high) {
              const auto& point_high = discretized_path.Evaluate(
                  high_s + discretized_path.StartPoint().s());
              if (!CheckOverlap(point_high, obs_box,
                                st_boundary_config_.boundary_buffer())) {
                high_s -= fine_tuning_step_length;
              } else {
                find_high = true;
              }
            }
          }
		 // 到此处得到某个障碍物预测轨迹点obs_box相对于curr_adc_path_point更加精细的障碍物标定框的上下界,这个上下界就是
		 // 某个时刻的障碍物预测位置标定框的上下界
		  
		  // 下面将上下界保存到lower_points和upper_points,作为
          if (find_high && find_low) {
            lower_points->emplace_back(
                low_s - st_boundary_config_.point_extension(),
                trajectory_point_time);
            upper_points->emplace_back(
                high_s + st_boundary_config_.point_extension(),
                trajectory_point_time);
          }
          break;
        }
      }
	  // 这里完成的是:根据障碍物预测轨迹点与期望路径上第一个发生碰撞的点,求取某个时刻障碍物预测轨迹点的上下界标定框
    }
	// 这里完成的是障碍物预测轨迹上所有预测轨迹点的上下界标定框,lower_points中存储的就是这个障碍物在所有时刻的标定框下界,
	// upper_points中存储的就是这个障碍物在所有时刻的标定框上界
  }
  DCHECK_EQ(lower_points->size(), upper_points->size());
  return (lower_points->size() > 1 && upper_points->size() > 1);
}

Status StBoundaryMapper::MapWithDecision(
    PathObstacle* path_obstacle, const ObjectDecisionType& decision) const {
  DCHECK(decision.has_follow() || decision.has_yield() ||
         decision.has_overtake())
      << "decision is " << decision.DebugString()
      << ", but it must be follow or yield or overtake.";

  std::vector<STPoint> lower_points;
  std::vector<STPoint> upper_points;
  // 基于障碍物path_obstacle->obstacle()的预测轨迹和期望路径path_points()来构建该障碍物轨迹上每一个预测轨迹点的
  // 标定框上下界。传入的参数:期望路径,路径上的障碍物,障碍物轨迹预测点标定框的上下界(用于存放结果)。如果成功找出
  // 了该障碍物的上下界标定框就返回true
  if (!GetOverlapBoundaryPoints(path_data_.discretized_path().path_points(),
                                *(path_obstacle->obstacle()), &upper_points,
                                &lower_points)) {
    return Status::OK();
  }

  // 对于纵向决策标签为跟随,并且该障碍物下界点的最后一个点的时间戳小于规划总时间(说明该障碍物的预测轨迹都在本周期规划的期望
  // 路径范围之内)的障碍物的处理。这么处理的原因是,障碍物预测轨迹的总时长只有5s,而规划时间planning_time_ = 8s,所以有可能出现
  // 障碍物最后上下界点的时刻是小于8s的,那么这个时候需要将后面一直到8s的障碍物上下界点补齐,才能保证这个障碍物被跟随
  if (decision.has_follow() && lower_points.back().t() < planning_time_) {
    const double diff_s = lower_points.back().s() - lower_points.front().s();
    const double diff_t = lower_points.back().t() - lower_points.front().t();
    // 计算在t = planning_time_ 时刻的障碍物的上下界点,并存储
    double extend_lower_s =
        diff_s / diff_t * (planning_time_ - lower_points.front().t()) +
        lower_points.front().s();
    const double extend_upper_s =
        extend_lower_s + (upper_points.back().s() - lower_points.back().s()) +
        1.0;
    upper_points.emplace_back(extend_upper_s, planning_time_);
    lower_points.emplace_back(extend_lower_s, planning_time_);
  }
  
  // 生成该障碍物的st边界框
  auto boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                      .ExpandByS(boundary_s_buffer)
                      .ExpandByT(boundary_t_buffer);

  // get characteristic_length and boundary_type.
  StBoundary::BoundaryType b_type = StBoundary::BoundaryType::UNKNOWN;
  double characteristic_length = 0.0;
  // 标定障碍物边界框的类型
  // 1. 跟随边界框
  if (decision.has_follow()) {
    characteristic_length = std::fabs(decision.follow().distance_s());
    b_type = StBoundary::BoundaryType::FOLLOW;
  } 
  // 2. 让行边界框
  else if (decision.has_yield()) {
    characteristic_length = std::fabs(decision.yield().distance_s());
    boundary = StBoundary::GenerateStBoundary(lower_points, upper_points)
                   .ExpandByS(characteristic_length);
    b_type = StBoundary::BoundaryType::YIELD;
  }
  //3. 超车边界框
  else if (decision.has_overtake()) {
    characteristic_length = std::fabs(decision.overtake().distance_s());
    b_type = StBoundary::BoundaryType::OVERTAKE;
  } else {
    DCHECK(false) << "Obj decision should be either yield or overtake: "
                  << decision.DebugString();
  }
  boundary.SetBoundaryType(b_type);
  // 边界框的id和障碍物的id一致
  boundary.SetId(path_obstacle->obstacle()->Id());
  boundary.SetCharacteristicLength(characteristic_length);
  path_obstacle->SetStBoundary(boundary);

  return Status::OK();
}

bool StBoundaryMapper::CheckOverlap(const PathPoint& path_point,
                                    const Box2d& obs_box,
                                    const double buffer) const {
  double left_delta_l = 0.0;
  double right_delta_l = 0.0;
  if (is_change_lane_) {
  	// 大于0,说明车辆偏向reference_line的左边,小于0,偏向右边
    if ((adc_sl_boundary_.start_l() + adc_sl_boundary_.end_l()) / 2.0 > 0.0) {
      // change to right
      left_delta_l = 1.0;
    } else {
      // change to left
      right_delta_l = 1.0;
    }
  }
  // 车辆的后轴中心到几何中心的向量
  Vec2d vec_to_center =
      Vec2d((vehicle_param_.front_edge_to_center() -
             vehicle_param_.back_edge_to_center()) /
                2.0,
            (vehicle_param_.left_edge_to_center() + left_delta_l -
             vehicle_param_.right_edge_to_center() + right_delta_l) /
                2.0)
          .rotate(path_point.theta());
  // 车辆在path_point这个点的时候的几何中心位置
  Vec2d center = Vec2d(path_point.x(), path_point.y()) + vec_to_center;
  // 以车辆几何中心和车长车宽以及车辆航向构建的box
  const Box2d adc_box =
      Box2d(center, path_point.theta(), vehicle_param_.length() + 2 * buffer,
            vehicle_param_.width() + 2 * buffer);
  return obs_box.HasOverlap(adc_box);
}

}  // namespace planning
}  // namespace apollo

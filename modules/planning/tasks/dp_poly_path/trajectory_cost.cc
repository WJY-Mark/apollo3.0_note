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

#include "modules/planning/tasks/dp_poly_path/trajectory_cost.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "modules/common/proto/pnc_point.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/vec2d.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Sigmoid;
using apollo::common::math::Vec2d;

TrajectoryCost::TrajectoryCost(
    const DpPolyPathConfig &config, const ReferenceLine &reference_line,
    const bool is_change_lane_path,
    const std::vector<const PathObstacle *> &obstacles,
    const common::VehicleParam &vehicle_param,
    const SpeedData &heuristic_speed_data, const common::SLPoint &init_sl_point)
    : config_(config),
      reference_line_(&reference_line),
      is_change_lane_path_(is_change_lane_path),
      vehicle_param_(vehicle_param),
      heuristic_speed_data_(heuristic_speed_data),
      init_sl_point_(init_sl_point) {
      // 总的时间是heuristic_speed_data_的总时间，是因为在本周期内考虑的也就是这个时间段之内的情景
  const float total_time =
      std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);
   //  config.eval_time_interval()是障碍物预测中所使用的时间步长=0.1s
  num_of_time_stamps_ = static_cast<uint32_t>(
      std::floor(total_time / config.eval_time_interval()));
   // 遍历每一个障碍物
  for (const auto *ptr_path_obstacle : obstacles) {
  	// 如果是无人车可忽略的障碍物，这里就不需要考虑。因为其对无人车前进无影响；
    if (ptr_path_obstacle->IsIgnore()) {
      continue;
    } 
    //如果是需要停车的障碍物也在这里不考虑，因为它会迫使车辆直接停车
	else if (ptr_path_obstacle->LongitudinalDecision().has_stop()) {
      continue;
    }
    const auto &sl_boundary = ptr_path_obstacle->PerceptionSLBoundary();
	// 车辆左边沿的l坐标
    const float adc_left_l =
        init_sl_point_.l() + vehicle_param_.left_edge_to_center();
	// 车辆右边沿的l坐标
    const float adc_right_l =
        init_sl_point_.l() - vehicle_param_.right_edge_to_center();
      // 说明障碍物在车辆的左边或者右边，横向间距比较大不会影响到车辆前进，那么这个障碍物也不必考虑
    if (adc_left_l + FLAGS_lateral_ignore_buffer < sl_boundary.start_l() ||
        adc_right_l - FLAGS_lateral_ignore_buffer > sl_boundary.end_l()) {
      continue;
    }

    const auto *ptr_obstacle = ptr_path_obstacle->obstacle();
    bool is_bycycle_or_pedestrian =
        (ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         ptr_obstacle->Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);
    // 如果障碍物是虚拟障碍物，那么也不用在这里考虑
    if (Obstacle::IsVirtualObstacle(ptr_obstacle->Perception())) {
      // Virtual obstacle
      continue;
    }
	// 如果障碍物是静止障碍物或者是自行车或行人,那么将其加入到静止障碍物列表
	else if (Obstacle::IsStaticObstacle(ptr_obstacle->Perception()) ||
               is_bycycle_or_pedestrian) {
      static_obstacle_sl_boundaries_.push_back(std::move(sl_boundary));
    } else {
      std::vector<Box2d> box_by_time;
      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
	  	// 计算动态障碍物在时间t*eval_time_interval()时间点的位置
        TrajectoryPoint trajectory_point =
            ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());
          // 将动态障碍物的位置坐标扩展为box
        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
        constexpr float kBuff = 0.5;
		// 将障碍物的box扩展
        Box2d expanded_obstacle_box =
            Box2d(obstacle_box.center(), obstacle_box.heading(),
                  obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
		// 将每个时刻的box存入到列表
        box_by_time.push_back(expanded_obstacle_box);
      }
	  // 每个动态障碍物在未来5s内,每一个时刻的位置box存入到列表
      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));
    }
  }
}

// 计算拟合曲线curve的pathcost
ComparableCost TrajectoryCost::CalculatePathCost(
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s, const uint32_t curr_level, const uint32_t total_level) {
  ComparableCost cost;
  float path_cost = 0.0;
  std::function<float(const float)> quasi_softmax = [this](const float x) {
    const float l0 = this->config_.path_l_cost_param_l0();//1.5
    const float b = this->config_.path_l_cost_param_b();//0.4
    const float k = this->config_.path_l_cost_param_k();//1.5
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0))); // 值阈在[0.4,1.0]之间,单减函数,l=0取值为0.94279
  };

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  const float width = vehicle_config.vehicle_param().width();
   // path_resolution: 1.0m，在两个点拟合的曲线上每1m采样一个点，每采一个点计算改点的path_cost
  for (float curve_s = 0.0; curve_s < (end_s - start_s);curve_s += config_.path_resolution()) {
      // 计算累计距离为curve_s处的横向偏移l
	const float l = curve.Evaluate(0, curve_s);
      // 横向偏移距离l的开销，path_l_cost：6.5
      // quasi_softmax(std::fabs(l))这个的作用要好好研究一下
    path_cost += l * l * config_.path_l_cost() * quasi_softmax(std::fabs(l));

    double left_width = 0.0;
    double right_width = 0.0;
	// 求取在curve_s处的道路左右宽度
    reference_line_->GetLaneWidth(curve_s + start_s, &left_width, &right_width);

    constexpr float kBuff = 0.2;
	// 非变道path 且 超出道路左边沿或者右边沿
    if (!is_change_lane_path_ && (l + width / 2.0 + kBuff > left_width ||
                                  l - width / 2.0 - kBuff < -right_width)) {
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
    }
     // 横向速度dl的cost
    const float dl = std::fabs(curve.Evaluate(1, curve_s));
    path_cost += dl * dl * config_.path_dl_cost();
    // 横向加速度ddl的cost
    const float ddl = std::fabs(curve.Evaluate(2, curve_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  // 到此处计算完毕两层之间的一条拟合曲线的cost
  
  // 这里乘以config_.path_resolution()是为了避免不同采样间隔造成的不统一,相当于把本段曲线的路径cost进行了归一化
  path_cost *= config_.path_resolution();

  if (curr_level == total_level) {
  	// 计算拟合曲线最后一个点处的横向偏移l
    const float end_l = curve.Evaluate(0, end_s - start_s);
    path_cost +=
        std::sqrt(end_l - init_sl_point_.l() / 2.0) * config_.path_end_l_cost();
  }
  cost.smoothness_cost = path_cost;
  return cost;
}

// 计算拟合曲线curve的StaticObstacleCost
ComparableCost TrajectoryCost::CalculateStaticObstacleCost(
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s) {
    // 计算静态障碍物的cost
  ComparableCost obstacle_cost;
  // 在拟合的曲线上每隔1m取一个点
  for (float curr_s = start_s; curr_s <= end_s;curr_s += config_.path_resolution()) {
      // 计算累计距离为curve_s处的横向偏移curr_l
	const float curr_l = curve.Evaluate(0, curr_s - start_s);
	  // 遍历每一个障碍物,计算总的静态障碍物cost
    for (const auto &obs_sl_boundary : static_obstacle_sl_boundaries_) {
      obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs_sl_boundary);
    }
  }
  obstacle_cost.safety_cost *= config_.path_resolution();
  return obstacle_cost;
}

//计算动态障碍物的cost。在进入到这个函数之前,我们已经对障碍物进行时间段运动采样，得到了障碍物每隔0.1s的坐标位置,
// 那么只要需要计算拟合曲线curve每个采样点和障碍物运动的坐标位置cost，求和就可以每个动态障碍物的cost。由于障碍物运动
// 坐标是每隔0.1s的时间进行采样的,所以这里curve采样和静态障碍物cost计算按照1m的距离间隔采样不同，这里curve也采用每隔0.1s
// 的时间采样,这样才能计算每一个时刻障碍物运动坐标点与curves上的点的相对位置关系,并且在curve上采样的次数和动态障碍物位置坐标
// 预测的次数是一样的

ComparableCost TrajectoryCost::CalculateDynamicObstacleCost(
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s) const {
  ComparableCost obstacle_cost;
  float time_stamp = 0.0;
  // num_of_time_stamps_是动态障碍物在未来一段时间 位置预测的次数,每次隔0.1s预测一次,所以curve也要每隔0.1s采样num_of_time_stamps_次
  for (size_t index = 0; index < num_of_time_stamps_;++index, time_stamp += config_.eval_time_interval()) {
    common::SpeedPoint speed_point;
	// 这里为什么要从heuristic_speed_data_中来求取speed_point的s,是因为,在time_stamp对应的点可能不再当前拟合曲线范围内,
	// 所以从本周期的heuristic_speed_data_中来求取speed_point的s
	//求取在time_stamp处的speed_point
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    // 求取当前speed_point相对于init_point的s
    float ref_s = speed_point.s() + init_sl_point_.s();
	// ref_s < start_s说明在当前time_stamp处，车辆还没有走到当前拟合曲线的起始点,也就是说车辆在未到达本段拟合曲线curve时,
	// 本段拟合cutve与障碍物的位置关系不会影响到车辆路径的生成,所以continue考虑下一个时刻
    if (ref_s < start_s) {
      continue;
    }
	// ref_s < start_s说明在当前time_stamp处，车辆已经走过了拟合曲线curve的终点,那么在这个time_stamp之后,这段拟合曲线与障碍物的
	// 位置关系就不需要再考虑了,因为肯定撞不了
    if (ref_s > end_s) {
      break;
    }
    // 运行到这里说明在当前time_stamp时刻,车辆能够进入拟合曲线curve范围内,需要考虑这个时刻curve上的采样点(也就是车辆在time_stamp
    // 时刻可能到达的点)与障碍物的位置关系
    const float s = ref_s - start_s;  // s on spline curve 相对于拟合曲线curve的start_s的s
    const float l = curve.Evaluate(0, s);
    const float dl = curve.Evaluate(1, s);

    const common::SLPoint sl = common::util::MakeSLPoint(ref_s, l);
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);// 当前时刻,车辆的box
    // 计算当前time_stamp时刻动态障碍物cost
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_) {
      obstacle_cost +=
          GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  constexpr float kDynamicObsWeight = 1e-6;
  obstacle_cost.safety_cost *=
      (config_.eval_time_interval() * kDynamicObsWeight);
  return obstacle_cost;
}

ComparableCost TrajectoryCost::GetCostFromObsSL(
   // 这里计算静态障碍物的基本思想就是:在利用相邻两个level的采样点拟合得到的曲线上进行采样,然后将车辆的box(adc_box)置于
   // 每一个采样点上,然后计算adc_box与每一个静态障碍物的adc_box的相对位置关系,然后根据计算得到的位置关系计算每一个采样点
   // 的静态障碍物cost
    const float adc_s, const float adc_l, const SLBoundary &obs_sl_boundary) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ComparableCost obstacle_cost;

  const float adc_front_s = adc_s + vehicle_param.front_edge_to_center();
  const float adc_end_s = adc_s - vehicle_param.back_edge_to_center();
  const float adc_left_l = adc_l + vehicle_param.left_edge_to_center();
  const float adc_right_l = adc_l - vehicle_param.right_edge_to_center();
    // 障碍物在车辆的左边或者右边，都不会影响车辆前进，这个障碍物的cost就是默认的obstacle_cost
  if (adc_left_l + FLAGS_lateral_ignore_buffer < obs_sl_boundary.start_l() ||
      adc_right_l - FLAGS_lateral_ignore_buffer > obs_sl_boundary.end_l()) {
    return obstacle_cost;
  }
  // 车辆的adc_box 与障碍物的box在纵向上或者横向上不重叠,no_overlap为真
  bool no_overlap = ((adc_front_s < obs_sl_boundary.start_s() ||
                      adc_end_s > obs_sl_boundary.end_s()) ||  // longitudinal纵向上不重叠
                     (adc_left_l + FLAGS_static_decision_nudge_l_buffer <
                          obs_sl_boundary.start_l() ||
                      adc_right_l - FLAGS_static_decision_nudge_l_buffer >
                          obs_sl_boundary.end_l()));  // lateral横向上不重叠

 //如果车辆和障碍物在纵向或者横向上重叠
  if (!no_overlap) {
    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
  }

  // if obstacle is behind ADC, ignore its cost contribution.
  // 条件满足说明障碍物在车辆后面
  // 感觉这里有点问题,我觉得应该是adc_end_s > obs_sl_boundary.end_s()时,障碍物在车辆的后边
  if (adc_front_s > obs_sl_boundary.end_s()) {
    return obstacle_cost;
  }
  // 车辆中心和障碍物中心的横向距离
  const float delta_l = std::fabs(
      adc_l - (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) / 2.0);

  const double kSafeDistance = 1.0;
    // 当车辆与障碍物中心的横向距离小于一定阈值时，计算横向偏差造成的cost
    // Sigmoid单增函数，所以当delta_l越小，cost越大,也就是说车辆和障碍物中心横向距离越小,cost的越大
  if (delta_l < kSafeDistance) {
    obstacle_cost.safety_cost +=
        config_.obstacle_collision_cost() *  // 系数，obstacle_collision_cost：1e8
        Sigmoid(config_.obstacle_collision_distance() - delta_l);// obstacle_collision_distance：0.5
  }

   // 静态障碍物中心和无人车中心前进方向上的距离
  const float delta_s = std::fabs(
      adc_s - (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) / 2.0);
  //  前方向距离造成的cost,Sigmoid单增函数，当delta_s越小，cost越大，即是车辆与障碍物的纵向距离越小,cost越大
  obstacle_cost.safety_cost +=
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - delta_s);
  return obstacle_cost;
}

// Simple version: calculate obstacle cost by distance
// 根据车辆的box:ego_box和动态障碍物列表来计算ego_box所在的curve上采样点的动态障碍物cost
ComparableCost TrajectoryCost::GetCostBetweenObsBoxes(
    const Box2d &ego_box, const Box2d &obstacle_box) const {
  ComparableCost obstacle_cost;
  // 动态障碍物box与车辆ego_box的距离
  const float distance = obstacle_box.DistanceTo(ego_box);
  //如果distance大于阈值说明障碍物不影响路径生成
  if (distance > config_.obstacle_ignore_distance()) {
    return obstacle_cost;
  }
  // 计算碰撞cost
  obstacle_cost.safety_cost +=  
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - distance);
  // 计算风险cost
  obstacle_cost.safety_cost +=
      20.0 * Sigmoid(config_.obstacle_risk_distance() - distance);
  return obstacle_cost;
}

Box2d TrajectoryCost::GetBoxFromSLPoint(const common::SLPoint &sl,
                                        const float dl) const {
  Vec2d xy_point;
  reference_line_->SLToXY(sl, &xy_point);

  ReferencePoint reference_point = reference_line_->GetReferencePoint(sl.s());

  const float one_minus_kappa_r_d = 1 - reference_point.kappa() * sl.l();
  const float delta_theta = std::atan2(dl, one_minus_kappa_r_d);
  const float theta =
      common::math::NormalizeAngle(delta_theta + reference_point.heading());
  return Box2d(xy_point, theta, vehicle_param_.length(),
               vehicle_param_.width());
}

// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,
                                         const float start_s, const float end_s,
                                         const uint32_t curr_level,
                                         const uint32_t total_level) {
  ComparableCost total_cost;
  // path cost
  total_cost +=
      CalculatePathCost(curve, start_s, end_s, curr_level, total_level);

  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  return total_cost;
}

}  // namespace planning
}  // namespace apollo

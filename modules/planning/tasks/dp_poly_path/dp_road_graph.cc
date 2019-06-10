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
 * @file dp_road_graph.h
 **/

#include "modules/planning/tasks/dp_poly_path/dp_road_graph.h"

#include <algorithm>
#include <utility>

#include "modules/common/proto/error_code.pb.h"
#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/planning_internal.pb.h"
#include "modules/planning/proto/planning_status.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/math/cartesian_frenet_conversion.h"
#include "modules/common/util/util.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/path/frenet_frame_path.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/planning_thread_pool.h"
#include "modules/planning/common/planning_util.h"
#include "modules/planning/math/curve1d/quintic_polynomial_curve1d.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::SLPoint;
using apollo::common::math::CartesianFrenetConverter;
using apollo::common::util::MakeSLPoint;

// 构造函数，读取配置文件中的信息，以及reference_line中的信息
DPRoadGraph::DPRoadGraph(const DpPolyPathConfig &config,
                         const ReferenceLineInfo &reference_line_info,
                         const SpeedData &speed_data)
    : config_(config),
      reference_line_info_(reference_line_info),
      reference_line_(reference_line_info.reference_line()),
      speed_data_(speed_data) {}

bool DPRoadGraph::FindPathTunnel(
    const common::TrajectoryPoint &init_point,
    const std::vector<const PathObstacle *> &obstacles,
    PathData *const path_data) {
  CHECK_NOTNULL(path_data);
  // 规划的起始点 init_point_(可能是缝合轨迹的最后一个点或者车辆当前位置点) 转换为sl点，
  // s，l为相对于reference_line的累加距离和横向偏差
  init_point_ = init_point;
  if (!reference_line_.XYToSL(
          {init_point_.path_point().x(), init_point_.path_point().y()},
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }
  // 将init_point_转换为frenet坐标点,计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向加速度ddl
  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
    AERROR << "Fail to create init_frenet_frame_point_ from : "
           << init_point_.DebugString();
    return false;
  }
  // 获取当前参考线下最优的前进路线min_cost_path 
  std::vector<DPRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
    AERROR << "Fail to generate graph!";
    return false;
  }
  // 将最优前进路线封装成path_data,路径点的形式为FrenetFramePoint(s ,l,dl,ddl)
  std::vector<common::FrenetFramePoint> frenet_path;
  float accumulated_s = init_sl_point_.s();
  const float path_resolution = config_.path_resolution();

  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

	// 前后两点之间的路径长度
    const float path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    float current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
    while (current_s + path_resolution / 2.0 < path_length) {
      const float l = curve.Evaluate(0, current_s);
      const float dl = curve.Evaluate(1, current_s);
      const float ddl = curve.Evaluate(2, current_s);
      common::FrenetFramePoint frenet_frame_point;
      frenet_frame_point.set_s(accumulated_s + current_s);
      frenet_frame_point.set_l(l);
      frenet_frame_point.set_dl(dl);
      frenet_frame_point.set_ddl(ddl);
      frenet_path.push_back(std::move(frenet_frame_point));
      current_s += path_resolution;
    }
    if (i == min_cost_path.size() - 1) {
      accumulated_s += current_s;
    } else {
      accumulated_s += path_length;
    }
  }
  FrenetFramePath tunnel(frenet_path);
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(tunnel);
  return true;
}

bool DPRoadGraph::GenerateMinCostPath(
    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DPRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);

  // 获取采样点存储在path_waypoints中,path_waypoints可以看成一个表格,这个表格的每一行(层)就是一个纵向采样层,
  // 每一行（也就是每一个纵向采样层）中的路径点就是在该纵向采样层上的横向采样点
  std::vector<std::vector<common::SLPoint>> path_waypoints;
  if (!SamplePathWaypoints(init_point_, &path_waypoints) ||
      path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = "
           << reference_line_.Length();
    return false;
  }
  // 完成采样,存储在了path_waypoints中,在path_waypoints的最开头再加一层,这一层只有一个点,即规划的起始点
  path_waypoints.insert(path_waypoints.begin(),
                        std::vector<common::SLPoint>{init_sl_point_});
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();

  TrajectoryCost trajectory_cost(
      config_, reference_line_, reference_line_info_.IsChangeLanePath(),
      obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

  // 构造一个网格，N个level，每个level若干个横向采样点，相邻两层level之间的采样点两两互相连接。
  // 而且网络中的每个node，都保存了从规划起始点到该节点的最小cost，以及反向连接链(cost最小对应的parent)
  std::list<std::list<DPRoadGraphNode>> graph_nodes;
  
  graph_nodes.emplace_back(); // 插入第一行

  // 第一层(规划起始点)加入网络图
  // 第一行中插入第一个元素,这里调用DPRoadGraphNode的第三个构造函数; 进而也会调用ComparableCost类的默认构造函数
  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost()); 

  // 取出graph_nodes表格的第一个元素,其实这个node就包含第一层的那个init_point到它本身的cost
  auto &front = graph_nodes.front().front();
  
  // 总的层数,应该等于纵向采样层的层数,也即是path_waypoints的行数
  size_t total_level = path_waypoints.size();
  // 遍历每一层,计算相邻两两level之间的cost
  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {
  	// 取出graph_nodes中的最后一层,也即是当前层的上一层
    const auto &prev_dp_nodes = graph_nodes.back();
	// 取出当前层
    const auto &level_points = path_waypoints[level];
    // graph_nodes中加入新的一行,初值为默认值,新加入的这一行的目的是为了存储根据当前level计算的node
    graph_nodes.emplace_back();
    // 遍历取出的当前层level_points的所有路点,计算当前level中的每一个点和前一层level中每个点两两之间的cost
    for (size_t i = 0; i < level_points.size(); ++i) {
	  // 取出当前层的第i个点
      const auto &cur_point = level_points[i];
      // 在graph_nodes的最新一行(即是上面新加入的默认值的那一行)中加入当前取出的点
      graph_nodes.back().emplace_back(cur_point, nullptr);
	  // 现在取出在graph_nodes中最新一行新加入的这个点
      auto &cur_node = graph_nodes.back().back();
      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        PlanningThreadPool::instance()->Push(std::bind(
            &DPRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,
            total_level, &trajectory_cost, &(front), &(cur_node)));

      } else {
	  	// 计算前一层prev_dp_nodes和当前层的节点cur_node的开销cost，取prev_dp_nodes中与cur_node开销cost最小的节点，
	  	//设置为最优路径
	  	//(当前层的上一层, 当前层的序号,总层数,trajectory_cost, front(对应于init_point), cur_node)
        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front,
                   &cur_node);
      }
    }
    if (FLAGS_enable_multi_thread_in_dp_poly_path) {
      PlanningThreadPool::instance()->Synchronize();
    }
  }

  // find best path
  DPRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  if (min_cost_node != &graph_nodes.front().front()) {
    return false;
  }

  std::reverse(min_cost_path->begin(), min_cost_path->end());

  for (const auto &node : *min_cost_path) {
    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
    planning_debug_->mutable_planning_data()
        ->mutable_dp_poly_graph()
        ->add_min_cost_point()
        ->CopyFrom(node.sl_point);
  }
  return true;
}

void DPRoadGraph::UpdateNode(const std::list<DPRoadGraphNode> &prev_nodes,
                             const uint32_t level, const uint32_t total_level,
                             TrajectoryCost *trajectory_cost,
                             DPRoadGraphNode *front,
                             DPRoadGraphNode *cur_node) {
  DCHECK_NOTNULL(trajectory_cost);
  DCHECK_NOTNULL(front);
  DCHECK_NOTNULL(cur_node);
  // 遍历上一层prev_nodes中的每一个node,求取prev_nodes中每一个node与当前的cur_node之间的拟合曲线
  for (const auto &prev_dp_node : prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = cur_node->sl_point;
    float init_dl = 0.0;
    float init_ddl = 0.0;
	// 如果不是level=1,那么上一层的点也是采样得到的,所以dl，ddl未知,所以才有上面的init_dl = 0.0,init_ddl = 0.0;
	// 如果时level = 1,那么上一点就是规划起始点init_point,这个点的dl,ddl是已知的,
    if (level == 1) {
      init_dl = init_frenet_frame_point_.dl();
      init_ddl = init_frenet_frame_point_.ddl();
    }
	// 前一个点prev_sl_point与当前点cur_point拟合出一条曲线curve
    QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl,
                                   cur_point.l(), 0.0, 0.0,
                                   cur_point.s() - prev_sl_point.s());
    // 判断拟合出的曲线是否有效,如果拟合的曲线无效,那么取上一层的下一个点再次进行拟合
    if (!IsValidCurve(curve)) {
      continue;
    }
	// 如果拟合出的曲线是有效的
    const auto cost =
         // Calculate(...)函数计算的是的上一层点prev_sl_point与当前点cur_point的cost
        trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                   level, total_level) +   
        // 再加上prev_dp_node.min_cost,得到的是当前点cur_point与init_point之间的cost
        prev_dp_node.min_cost;
    // 获取上一层所有的node中到当前层当前cur_node的最小cost的那个node,存储到min_cost_prev_node
    cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }
  // 这个循环结束,求取到了上一层所有node中到cur_node最小cost的点min_cost_prev_node。这个cost考虑了从init_point到上一层
  // 每个node的cost

  
  // try to connect the current point with the first point directly
  if (level >= 2) {
    const float init_dl = init_frenet_frame_point_.dl();
    const float init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(init_sl_point_.l(), init_dl, init_ddl,
                                   cur_node->sl_point.l(), 0.0, 0.0,
                                   cur_node->sl_point.s() - init_sl_point_.s());
    // 如果曲
	if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = trajectory_cost->Calculate(
        curve, init_sl_point_.s(), cur_node->sl_point.s(), level, total_level);
    cur_node->UpdateCost(front, curve, cost);
  }
}

bool DPRoadGraph::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);
  // 计算路径点的采样距离
  const float kMinSampleDistance = 40.0;
  const float total_length = std::fmin(
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
      reference_line_.Length());
  // 获取车辆本身的几何参数
  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  // 半车宽
  const float half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
  // 每一层采样的数量
  const size_t num_sample_per_level =
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()// 3
                                : config_.sample_points_num_each_level(); // 7
  // 遍历obstacles，确认是否有可以绕行障碍物
  const bool has_sidepass = HasSidepass();
  // 采样点前向预瞄时长
  constexpr float kSamplePointLookForwardTime = 4.0;
  // 采样点预瞄步长
  const float step_length =
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());
  // 两个采样层之间的距离
  const float level_distance =
      (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;

  float accumulated_s = init_sl_point_.s();
  float prev_s = accumulated_s;

  auto *status = util::GetPlanningStatus();
  if (status == nullptr) {
    AERROR << "Fail to  get planning status.";
    return false;
  }
  // // 表示无人车已进入PULL_OVER可操作区域
  if (status->planning_state().has_pull_over() &&
      status->planning_state().pull_over().in_pull_over()) {
    status->mutable_planning_state()->mutable_pull_over()->set_status(
        PullOverStatus::IN_OPERATION);
    const auto &start_point =
        status->planning_state().pull_over().start_point();
    SLPoint start_point_sl;
    if (!reference_line_.XYToSL(start_point, &start_point_sl)) {
      AERROR << "Fail to change xy to sl.";
      return false;
    }

    if (init_sl_point_.s() > start_point_sl.s()) {
      const auto &stop_point =
          status->planning_state().pull_over().stop_point();
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {
        AERROR << "Fail to change xy to sl.";
        return false;
      }
	  // 这时候纵向上只要设置一个采样层level_points即可,并且在横向上该采样层只有一个采样点就是停车位置
      std::vector<common::SLPoint> level_points(1, stop_point_sl);
      points->emplace_back(level_points);
      return true;
    }
  }
  // 在采样总长度total_length范围内采样，采样的间隔是level_distance,即纵向上每隔level_distance采样一层
  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
	// 确保每一个纵向采样层的累计路径长度accumulated_s不会超过采样总长度
    const float s = std::fmin(accumulated_s, total_length);
	// 最小采样步长
    constexpr float kMinAllowedSampleStep = 1.0;
	// 如果当前采样层的累计路径长度比上一个采样层的累计长度之差小于最小采样长度的话,那么这一层不进行采样,开始下一层
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    double left_width = 0.0;
    double right_width = 0.0;
	// 获取从reference_line_累积距离为s的点到道路左右两边的距离
    reference_line_.GetLaneWidth(s, &left_width, &right_width);

	// 计算能够实际用于采样的道路左右宽度(与下面计算的横向采样左右边界不一样)
    constexpr float kBoundaryBuff = 0.20;
    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    // the heuristic shift of L for lane change scenarios
    // sl坐标系中的l坐标沿着s的变化率，这里设置为定值，代表没纵向上走20m,横向上的偏差减小1.2m
    const double delta_dl = 1.2 / 20.0;
	// 计算当前纵向采样层的横向偏差变化量
	// 比如,init_frenet_frame_point_.dl() = 0, level_distance = 25时,计算出kChangeLaneDeltaL = 1.5m,也就是说在该纵向
	// 采样层上,车辆与reference_line_的横向偏差最多减小1.5m
    const double kChangeLaneDeltaL = common::math::Clamp(
        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),
        1.2, 3.5);
    // 在某一个纵向采样层上，横向采样点间隔距离,
    // 接上面的例子,这个时候 1.5/(7-1) = 0.25m
    // 看到这里就会注意到,在每一纵向采样层上进行横向采样时,并不是尽可着道路左右宽度来采样,而是有一定限制的,这个限制和
    // init_frenet_frame_point_.dl(), delta_dl, level_distance都有关
    float kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);
   
	if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) { //如果当前参考线是变道，且变道不安全(无人车前后一定距离内有障碍物)
      kDefaultUnitL = 1.0;  //那么增大横向采样点间隔，这样可以下一时刻减少变道的时间。
    }
    // 计算横向采样距离(和上面说的不会尽可着道路宽度采样,对应起来了)
    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    // 横向采样的左右边界
    float sample_right_boundary = -eff_right_width;
    float sample_left_boundary = eff_left_width;

    const float kLargeDeviationL = 1.75;// 最大的横向偏差,半车道宽,

	// 如果当前reference_line_info_是变道,或者init_sl_point_ > kLargeDeviationL(表示本周期的reference_line_info_相对于上一个周期已经偏移到另外一个车道了)
    if (reference_line_info_.IsChangeLanePath() ||  
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());//采样右边界必须包含init_sl_point_在内(对应左变道,应把右边界外扩)
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());//采样左边界必须包含init_sl_point_在内(对应右变道,应把左边界外扩)

      if (init_sl_point_.l() > eff_left_width) {//右变道,上面已经外扩了左边界,这里要内缩右边界,这样才能保证横向采样宽度不变
        sample_right_boundary = std::fmax(sample_right_boundary,
                                          init_sl_point_.l() - sample_l_range);
      }
      if (init_sl_point_.l() < eff_right_width) {//左变道,上面已经外扩了右边界,这里要内缩左边界,这样才能保证横向采样宽度不变
        sample_left_boundary = std::fmin(sample_left_boundary,
                                         init_sl_point_.l() + sample_l_range);
      }
    }
    // 截至到此处,在本纵向采样层上横向采样的左右边界最终确定sample_left_boundary, sample_right_boundary(考虑了换道情况在内)

    // 开始进行采样

	//  如果当前参考线需要变道，并且变道不安全，那么横向采样点就设置为第二条参考线(即变道目标线)的位置,直接走第二条参考线。
	// 也就是说,这个时候每个level的横向采样只采一个点,且这个点位于变道目标线上。
    std::vector<float> sample_l;
    if (reference_line_info_.IsChangeLanePath() &&
        !reference_line_info_.IsSafeToChangeLane()) {
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
    }
	// 如果当前参考线需要侧方绕行(SIDEPASS)，即从障碍物旁边绕过。这个时候也是只在横向上采一个点,左绕行采最左边,右绕行采最右边
	// 若是可以进行左边超车，那么横向采样点设置为左边界+超车距离；右边超车，横向采样点设置为右边界+超车距离
	else if (has_sidepass) {
      // currently only left nudge is supported. Need road hard boundary for
      // both sides
      switch (sidepass_.type()) {
        case ObjectSidePass::LEFT: { // 向左绕行, 那么道路有效左边宽度eff_left_width加宽(再增加一个绕行宽度sidepass_distance)
          sample_l.push_back(eff_left_width + config_.sidepass_distance());
          break;
        }
        case ObjectSidePass::RIGHT: { // 向右绕行, 那么道路有效右边宽度eff_right_width加宽(再增加一个绕行宽度sidepass_distance)
          sample_l.push_back(-eff_right_width - config_.sidepass_distance());
          break;
        }
        default:
          break;
      }
    }
	// 正常行驶情况下，从横向区间[sample_right_boundary , sample_left_boundary]大小为sample_l_range进行均匀采样
	// 比如[-1.2,1.2],横向采七个点时就是[-1.2,-0.8,-0.4,0,0.4,0.8,1.2]
	else {
      common::util::uniform_slice(sample_right_boundary, sample_left_boundary,
                                  num_sample_per_level - 1, &sample_l);
    }
	// 计算每个横向采样点的相对偏移距离l和累计距离s，封装成一个level，最后所有level封装成way_points
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;

	// 每个纵向位置采样得到横向采样点，封装成一个level_points
    for (size_t j = 0; j < sample_l.size(); ++j) {
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
    }
	// 如果不换道但是需要绕行,那么再采集一个当前reference_line_info_上的点
    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);
      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(std::move(sl_zero));
    }

    if (!level_points.empty()) {// level不为空，封装进最终的way+points
      planning_debug_->mutable_planning_data()
          ->mutable_dp_poly_graph()
          ->add_sample_layer()
          ->CopyFrom(sample_layer_debug);
      points->emplace_back(level_points);
    }
	// 至此完成一个level的采样,并且存储到了points中
  }
  // 至此完成所有level的采样,并且存储到了points中
  return true;
}

bool DPRoadGraph::CalculateFrenetPoint(
    const common::TrajectoryPoint &traj_point,
    common::FrenetFramePoint *const frenet_frame_point) {
  common::SLPoint sl_point;
  if (!reference_line_.XYToSL(
          {traj_point.path_point().x(), traj_point.path_point().y()},
          &sl_point)) {
    return false;
  }
  frenet_frame_point->set_s(sl_point.s());
  frenet_frame_point->set_l(sl_point.l());

  const float theta = traj_point.path_point().theta();
  const float kappa = traj_point.path_point().kappa();
  const float l = frenet_frame_point->l();

  ReferencePoint ref_point;
  ref_point = reference_line_.GetReferencePoint(frenet_frame_point->s());

  const float theta_ref = ref_point.heading();
  const float kappa_ref = ref_point.kappa();
  const float dkappa_ref = ref_point.dkappa();

  const float dl = CartesianFrenetConverter::CalculateLateralDerivative(
      theta_ref, theta, l, kappa_ref);
  const float ddl =
      CartesianFrenetConverter::CalculateSecondOrderLateralDerivative(
          theta_ref, theta, kappa_ref, kappa, dkappa_ref, l);
  frenet_frame_point->set_dl(dl);
  frenet_frame_point->set_ddl(ddl);
  return true;
}

bool DPRoadGraph::IsValidCurve(const QuinticPolynomialCurve1d &curve) const {
  constexpr float kMaxLateralDistance = 20.0;
  // 从输入的曲线curve的起始点开始遍历,每隔2m取一个点,并求取其对应的横向偏差l
  // 如果l大于kMaxLateralDistance,说明该曲线无效
  for (float s = 0.0; s < curve.ParamLength(); s += 2.0) {
    const float l = curve.Evaluate(0, s);
    if (std::fabs(l) > kMaxLateralDistance) {
      return false;
    }
  }
  return true;
}

void DPRoadGraph::GetCurveCost(TrajectoryCost trajectory_cost,
                               const QuinticPolynomialCurve1d &curve,
                               const float start_s, const float end_s,
                               const uint32_t curr_level,
                               const uint32_t total_level,
                               ComparableCost *cost) {
  *cost =
      trajectory_cost.Calculate(curve, start_s, end_s, curr_level, total_level);
}

bool DPRoadGraph::HasSidepass() {
  const auto &path_decision = reference_line_info_.path_decision();
  for (const auto &obstacle : path_decision.path_obstacles().Items()) {
    if (obstacle->LateralDecision().has_sidepass()) {
      sidepass_ = obstacle->LateralDecision().sidepass();
      return true;
    }
  }
  return false;
}

}  // namespace planning
}  // namespace apollo

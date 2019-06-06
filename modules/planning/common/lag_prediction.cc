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

#include "modules/planning/common/lag_prediction.h"

#include <algorithm>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::adapter::AdapterManager;
using apollo::perception::PerceptionObstacle;
using apollo::prediction::PredictionObstacle;
using apollo::prediction::PredictionObstacles;

LagPrediction::LagPrediction(uint32_t min_appear_num,
                             uint32_t max_disappear_num)
    : min_appear_num_(min_appear_num), max_disappear_num_(max_disappear_num) {
  if (AdapterManager::GetPredictionConfig().message_history_limit() <
      static_cast<int32_t>(min_appear_num_)) {
    AWARN << "Prediction adapter history limit is "
          << AdapterManager::GetPredictionConfig().message_history_limit()
          << ", but an obstacle need to be observed at least "
          << min_appear_num_ << " times";
    return;
  }
}

void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
  obstacles->mutable_prediction_obstacle()->Clear();
  // 如果不能获取到预测信息或者预测信息为空的话，就直接退出
  if (!AdapterManager::GetPrediction() ||
      AdapterManager::GetPrediction()->Empty()) {
    return;
  }
  // 获取PredictionAdapter中所有已经发布的预测信息
  const auto& prediction = *(AdapterManager::GetPrediction());
  // 如果不能获取到定位信息，那么就和不启用滞后预测是一样的，直接使用预测结果
  if (!AdapterManager::GetLocalization() ||
      AdapterManager::GetLocalization()->Empty()) {  // no localization
    obstacles->CopyFrom(prediction.GetLatestObserved());
    return;
  }
  // 获取车辆定位信息
  const auto adc_position =
      AdapterManager::GetLocalization()->GetLatestObserved().pose().position();

  // 获取最新的预测信息
  // 最近一次发布的数据直接加入PredictionObstacles容器中
  const auto latest_prediction = (*prediction.begin());
  const double timestamp = latest_prediction->header().timestamp_sec();
  // 对最新一次获取的障碍物预测信息中的每一个预测障碍物进行处理
  std::unordered_set<int> protected_obstacles;
  for (const auto& obstacle : latest_prediction->prediction_obstacle()) {
    const auto& perception = obstacle.perception_obstacle();
	// 如果置信度小于阈值0.5，并且不是车辆障碍物，就不管它
    if (perception.confidence() < FLAGS_perception_confidence_threshold &&
        perception.type() != PerceptionObstacle::VEHICLE) {
      continue;
    }
	// 计算障碍物与车辆之间的距离
    double distance =
        common::util::DistanceXY(perception.position(), adc_position);
	// 如果障碍物与车辆之间的距离小于延迟预测阈值30m，那么就存储其id号到protected_obstacles，
	// 并将该障碍物存储到obstacles
    if (distance < FLAGS_lag_prediction_protection_distance) {//30
      protected_obstacles.insert(obstacle.perception_obstacle().id());
      // add protected obstacle to obstacles
      AddObstacleToPrediction(0.0, obstacle, obstacles);
    }
  }

  std::unordered_map<int, LagInfo> obstacle_lag_info;
  int index = 0;  // data in begin() is the most recent data
  for (auto it = prediction.begin(); it != prediction.end(); ++it, ++index) 
  {
    for (const auto& obstacle : (*it)->prediction_obstacle()) 
	{
      const auto& perception = obstacle.perception_obstacle();
      auto id = perception.id();
	  // 障碍物的置信度小于0.5并且不是车辆障碍物，不用处理
      if (perception.confidence() < FLAGS_perception_confidence_threshold &&
          perception.type() != PerceptionObstacle::VEHICLE) {
        continue;
      }
	  // 如果障碍物在最新一次发布的消息中出现了，那么不用管它，
	  // 因为在上面对最新一次障碍物预测信息处理中已经考虑过了
      if (protected_obstacles.count(id) > 0) {
        continue;  // don't need to count the already added protected obstacle
      }
	  // 运行到此处，说明该障碍物在最新一次发布的障碍物预测消息中没有出现，并且其置信度大于0.5或者为车辆障碍物
      auto& info = obstacle_lag_info[id];
	  // 该障碍物对应的计数器加1
      ++info.count;
	  // 在所有历史数据中，只保存该障碍物的最新信息 
      if ((*it)->header().timestamp_sec() > info.last_observed_time) {
        info.last_observed_time = (*it)->header().timestamp_sec();
        info.last_observed_seq = index;// index等于几，最新的障碍物信息就是在当前周期前几个周期出现，
                                       // 比如index=1，就是说这个障碍物的最新信息是在上一个周期出现的
        info.obstacle_ptr = &obstacle;
      }
    }
  }
  
  // 使用最新一次发布的障碍物预测信息中的header
  obstacles->mutable_header()->CopyFrom(latest_prediction->header());
  obstacles->mutable_header()->set_module_name("lag_prediction");
  obstacles->set_perception_error_code(
      latest_prediction->perception_error_code());
  obstacles->set_start_timestamp(latest_prediction->start_timestamp());
  obstacles->set_end_timestamp(latest_prediction->end_timestamp());

  // 计算障碍物预测消息buffer中帧的数量是否大于min_appear_num_
  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >=
                   static_cast<int32_t>(min_appear_num_);
  for (const auto& iter : obstacle_lag_info) {
  	// 历史信息中如果障碍物出现次数小于min_appear_num_/3次，次数太少，不能够确定是否误检，可忽略
    if (apply_lag && iter.second.count < min_appear_num_) {
      continue;
    }
	// 历史信息中如果障碍物最近一次发布的时间距离现在很远了，可以忽略，不再管这个障碍物。
    if (apply_lag && iter.second.last_observed_seq > max_disappear_num_) {
      continue;
    }
	// 运行到这里，排除了可以忽略的障碍物，那么剩下不可忽略的这些障碍物都需要在本规划周期加以考虑
    AddObstacleToPrediction(timestamp - iter.second.last_observed_time,//障碍物最新出现时间与当前时刻的时间差
                            *(iter.second.obstacle_ptr), obstacles);
  }
}

void LagPrediction::AddObstacleToPrediction(
    double delay_sec, const prediction::PredictionObstacle& history_obstacle,
    prediction::PredictionObstacles* obstacles) const {
  auto* obstacle = obstacles->add_prediction_obstacle();
  if (delay_sec <= 1e-6) {
    obstacle->CopyFrom(history_obstacle);
    return;
  }
  obstacle->mutable_perception_obstacle()->CopyFrom(
      history_obstacle.perception_obstacle());
  // 对某个障碍物的每一条预测轨迹进行考虑
  for (const auto& hist_trajectory : history_obstacle.trajectory()) {
    auto* traj = obstacle->add_trajectory();
	// 对某个预测轨迹的每一个点进行考虑
    for (const auto& hist_point : hist_trajectory.trajectory_point()) {
	// 如果轨迹上某个点的相对时间，小于delay_sec,那么这个点不用考虑，这是因为：delay_sec是该障碍物出现的
	// 最新时间(也就是相对时间的零点)与当前时间的差，如果相对时间小于delay_sec,说明该点的时间相对与车辆当前
	// 时间已经是过去了，我们不再关心，我们关注的是从当前时间往后，障碍物可能出现的位置，即障碍物未来的预测轨迹
      if (hist_point.relative_time() < delay_sec) {
        continue;
      }
      auto* point = traj->add_trajectory_point();
      point->CopyFrom(hist_point);
	  // 需要考虑的轨迹点的相对时间的零点更改为当前时刻
      point->set_relative_time(hist_point.relative_time() - delay_sec);
    }
	// 如果某个预测轨迹不存在,即相对于现在都已经是过去时的时候，那么这条轨迹我们就不在考虑了，直接考虑下一条轨迹
    if (traj->trajectory_point_size() <= 0) {
      obstacle->mutable_trajectory()->RemoveLast();
      continue;
    }
	// 每一条轨迹的概率仍然继承之前的概率
    traj->set_probability(hist_trajectory.probability());
  }
  // 如果某个障碍物的所有轨迹都不存在，那么这个障碍物直接就不用考虑了
  if (obstacle->trajectory_size() <= 0) {
    obstacles->mutable_prediction_obstacle()->RemoveLast();
    return;
  }
  // 每个障碍物的时间戳为其原来的时间戳
  obstacle->set_timestamp(history_obstacle.timestamp());
  obstacle->set_predicted_period(history_obstacle.predicted_period());
}

}  // namespace planning
}  // namespace apollo

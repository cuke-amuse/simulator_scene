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

#include <map>
#include <memory>
#include <string>

#include "modules/planning/proto/traffic_rule_config.pb.h"

// TODO(all) #include "modules/planning/navi_planning.h"
#include "modules/planning/on_lane_planning.h"
#include "modules/planning/planning_base.h"


namespace apollo {
namespace planning {

DECLARE_string(test_routing_response_file);
DECLARE_string(test_localization_file);
DECLARE_string(test_chassis_file);
DECLARE_string(test_data_dir);
DECLARE_string(test_prediction_file);
DECLARE_string(test_traffic_light_file);
DECLARE_string(test_relative_map_file);
DECLARE_string(test_previous_planning_file);

class PlanningTestBase{
 public:
  virtual ~PlanningTestBase() = default;

  static void SetUpTestCase();
  void SetUp();
  void UpdateData();

  /**
   * Execute the planning code.
   * @return true if planning is success. The ADCTrajectory will be used to
   * store the planing results.  Otherwise false.
   */
  bool RunPlanning(const std::string& test_case_name, int case_num,
                   bool no_trajectory_point);

  TrafficRuleConfig* GetTrafficRuleConfig(
      const TrafficRuleConfig::RuleId& rule_id);

 protected:
  void TrimPlanning(ADCTrajectory* origin, bool no_trajectory_point);
  bool FeedTestData();
  bool IsValidTrajectory(const ADCTrajectory& trajectory);

 public:
  std::unique_ptr<PlanningBase> planning_ = nullptr;
  std::map<TrafficRuleConfig::RuleId, bool> rule_enabled_;
  ADCTrajectory adc_trajectory_;
  LocalView local_view_;
  PlanningConfig config_;
  std::shared_ptr<DependencyInjector> injector_;
};

}  // namespace planning
}  // namespace apollo

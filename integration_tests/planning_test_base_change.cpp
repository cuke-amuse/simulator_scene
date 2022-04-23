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

#include "modules/planning/integration_tests/planning_test_base_change.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/prediction/proto/prediction_obstacle.pb.h"
#include "modules/routing/proto/routing.pb.h"

//#include "modules/visualization/matplotlibcpp/matplotlibcpp.h"
//namespace plt = matplotlibcpp;
namespace apollo {
namespace planning {

using apollo::canbus::Chassis;
using apollo::cyber::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::TrafficLightDetection;
using apollo::prediction::PredictionObstacles;
using apollo::routing::RoutingResponse;

DEFINE_string(test_data_dir, "", "the test data folder");
DEFINE_bool(test_update_golden_log, false,
            "true to update decision golden log file.");
DEFINE_string(test_routing_response_file, "", "The routing file used in test");
DEFINE_string(test_localization_file, "", "The localization test file");
DEFINE_string(test_chassis_file, "", "The chassis test file");
DEFINE_string(test_planning_config_file, "", "planning config file for test");
DEFINE_string(test_prediction_file, "", "The prediction module test file");
DEFINE_string(test_traffic_light_file, "", "The traffic light test file");
DEFINE_string(test_relative_map_file, "", "The relative map test file");
DEFINE_string(test_previous_planning_file, "",
              "The previous planning test file");

void PlanningTestBase::SetUpTestCase() {
  FLAGS_use_multi_thread_to_add_obstacles = false;
  FLAGS_enable_multi_thread_in_dp_st_graph = false;
  FLAGS_traffic_rule_config_filename =
      "/apollo/modules/planning/conf/traffic_rule_config.pb.txt";
  FLAGS_smoother_config_filename =
      "/apollo/modules/planning/conf/qp_spline_smoother_config.pb.txt";
  FLAGS_map_dir = "/apollo/modules/planning/testdata";
  FLAGS_test_localization_file = "";
  FLAGS_test_chassis_file = "";
  FLAGS_test_routing_response_file = "";
  FLAGS_test_planning_config_file =
      "/apollo/modules/planning/conf/planning_config.pb.txt";
  FLAGS_test_previous_planning_file = "";
  FLAGS_test_prediction_file = "";
  FLAGS_align_prediction_time = false;
  FLAGS_enable_reference_line_provider_thread = false;
  // FLAGS_enable_trajectory_check is temporarily disabled, otherwise EMPlanner
  // and LatticePlanner can't pass the unit test.
  FLAGS_enable_trajectory_check = false;
  FLAGS_planning_test_mode = true;
}

bool PlanningTestBase::FeedTestData() {
  // chassis
  Chassis chassis;
  if (FLAGS_test_chassis_file.empty()) {
    AERROR << "Requires FLAGS_test_chassis_file to be set";
    return false;
  }
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_chassis_file, &chassis)) {
    AERROR << "failed to load file: " << FLAGS_test_chassis_file;
    return false;
  }
  // localization
  if (FLAGS_test_localization_file.empty()) {
    AERROR << "Requires FLAGS_test_localization_file to be set";
    return false;
  }
  LocalizationEstimate localization;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_localization_file,
          &localization)) {
    AERROR << "failed to load file: " << FLAGS_test_localization_file;
    return false;
  }
  Clock::SetMode(apollo::cyber::proto::MODE_MOCK);
  Clock::SetNowInSeconds(localization.header().timestamp_sec());

  // prediction
  if (FLAGS_test_prediction_file.empty()) {
    AERROR << "Requires FLAGS_test_prediction_file to be set";
    return false;
  }
  PredictionObstacles prediction;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_prediction_file,
          &prediction)) {
    AERROR << "failed to load file: " << FLAGS_test_prediction_file;
    return false;
  }
  // routing_response
  if (FLAGS_test_routing_response_file.empty()) {
    AERROR << "Requires FLAGS_test_routing_response_file";
    return false;
  }
  RoutingResponse routing_response;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_routing_response_file,
          &routing_response)) {
    AERROR << "failed to load file: " << FLAGS_test_routing_response_file;
    return false;
  }
  // traffic_light_detection
  // optional
  TrafficLightDetection traffic_light_detection;
  if (!apollo::cyber::common::GetProtoFromFile(
          FLAGS_test_data_dir + "/" + FLAGS_test_traffic_light_file,
          &traffic_light_detection)) {
    AERROR << "failed to load file: " << FLAGS_test_traffic_light_file;
    return false;
  }

  local_view_.prediction_obstacles =
      std::make_shared<PredictionObstacles>(prediction);
  local_view_.chassis = std::make_shared<Chassis>(chassis);
  local_view_.localization_estimate =
      std::make_shared<LocalizationEstimate>(localization);
  local_view_.routing =
      std::make_shared<routing::RoutingResponse>(routing_response);
  local_view_.traffic_light =
      std::make_shared<TrafficLightDetection>(traffic_light_detection);

  AINFO << "Successfully feed proto files.";
  return true;
}

void PlanningTestBase::SetUp() {
  injector_ = std::make_shared<DependencyInjector>();

  if (FLAGS_use_navigation_mode) {
    // TODO(all)
    // planning_ = std::unique_ptr<PlanningBase>(new NaviPlanning());
  } else {
    //PlanningBase 由OnLanePlanning和NaviPlanning继承
    planning_ = std::unique_ptr<PlanningBase>(new OnLanePlanning(injector_));
  }
  //喂送数据
  ACHECK(FeedTestData()) << "Failed to feed test data";
  //获取config内容
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_test_planning_config_file,
                                         &config_))
      << "failed to load planning config file "
      << FLAGS_test_planning_config_file;

  ACHECK(planning_->Init(config_).ok()) << "Failed to init planning module";

  if (!FLAGS_test_previous_planning_file.empty()) {
    const auto prev_planning_file =
        FLAGS_test_data_dir + "/" + FLAGS_test_previous_planning_file;
    ADCTrajectory prev_planning;
    ACHECK(cyber::common::GetProtoFromFile(prev_planning_file, &prev_planning));
    planning_->last_publishable_trajectory_.reset(
        new PublishableTrajectory(prev_planning));
  }
  for (auto& config : *(planning_->traffic_rule_configs_.mutable_config())) {
    auto iter = rule_enabled_.find(config.rule_id());
    if (iter != rule_enabled_.end()) {
      config.set_enabled(iter->second);
    }
  }
}

void PlanningTestBase::UpdateData() {
  ACHECK(FeedTestData()) << "Failed to feed test data";

  if (!FLAGS_test_previous_planning_file.empty()) {
    const auto prev_planning_file =
        FLAGS_test_data_dir + "/" + FLAGS_test_previous_planning_file;
    ADCTrajectory prev_planning;
    ACHECK(cyber::common::GetProtoFromFile(prev_planning_file, &prev_planning));
    planning_->last_publishable_trajectory_.reset(
        new PublishableTrajectory(prev_planning));
  }
  for (auto& config : *planning_->traffic_rule_configs_.mutable_config()) {
    auto iter = rule_enabled_.find(config.rule_id());
    if (iter != rule_enabled_.end()) {
      config.set_enabled(iter->second);
    }
  }
}

void PlanningTestBase::TrimPlanning(ADCTrajectory* origin,
                                    bool no_trajectory_point) {
  origin->clear_latency_stats();
  origin->clear_debug();
  // origin->mutable_header()->clear_radar_timestamp();
  // origin->mutable_header()->clear_lidar_timestamp();
  // origin->mutable_header()->clear_timestamp_sec();
  // origin->mutable_header()->clear_camera_timestamp();
  // origin->mutable_header()->clear_sequence_num();

  if (no_trajectory_point) {
    origin->clear_total_path_length();
    origin->clear_total_path_time();
    origin->clear_trajectory_point();
  }
}

bool PlanningTestBase::RunPlanning(const std::string& test_case_name,
                                   int case_num, bool no_trajectory_point) {
  //here we can use auto ...
  //to create node
  std::unique_ptr<apollo::cyber::Node> node;
  //auto node = apollo::cyber::CreateNode("planning_example");
  node = apollo::cyber::CreateNode("planning_example");

  //to create planning_writer
  std::shared_ptr<apollo::cyber::Writer<ADCTrajectory>> planning_writer;
  planning_writer = node->CreateWriter<ADCTrajectory>(
                "/apollo/planning");

  //to create localization_writer
  std::shared_ptr<apollo::cyber::Writer<LocalizationEstimate>> localization_writer;
  localization_writer =
      node->CreateWriter<LocalizationEstimate>("/apollo/localization/pose");

  //to create PredictionObstacles_writer
  std::shared_ptr<apollo::cyber::Writer<PredictionObstacles>> PredictionObstacles_writer;
  PredictionObstacles_writer =
      node->CreateWriter<PredictionObstacles>("/apollo/prediction");
  //to create chassis_writer
  std::shared_ptr<apollo::cyber::Writer<Chassis>> Chassis_writer;
  Chassis_writer =
      node->CreateWriter<Chassis>("/apollo/canbus/chassis ");

  //to create routing reponse
  std::shared_ptr<apollo::cyber::Writer<RoutingResponse>> RoutingResponse_writer;
  RoutingResponse_writer =
      node->CreateWriter<RoutingResponse>("/apollo/routing_response");
  //to define publish topic rate
  apollo::cyber::Rate rate(5.0);

  const std::string golden_result_file =
      absl::StrCat("result_", test_case_name, "_", case_num, ".pb.txt");

  std::string full_golden_path = FLAGS_test_data_dir + "/" + golden_result_file;

  ADCTrajectory adc_trajectory_pb;
  planning_->RunOnce(local_view_, &adc_trajectory_pb);
  int i = 10;
  while (apollo::cyber::OK()) {
#if 1
    planning_->RunOnce(local_view_, &adc_trajectory_pb);
    local_view_.localization_estimate->mutable_pose()->mutable_position()->set_x(adc_trajectory_pb.trajectory_point().at(i).path_point().x());
    local_view_.localization_estimate->mutable_pose()->mutable_position()->set_y(adc_trajectory_pb.trajectory_point().at(i).path_point().y());
    local_view_.localization_estimate->mutable_pose()->set_heading(adc_trajectory_pb.trajectory_point().at(i).path_point().theta());
    local_view_.localization_estimate->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    local_view_.chassis->mutable_header()->set_timestamp_sec(Clock::NowInSeconds());
    local_view_.chassis->set_speed_mps(adc_trajectory_pb.trajectory_point().at(i).v());
#endif
    localization_writer->Write(local_view_.localization_estimate);
    planning_writer->Write(adc_trajectory_pb);
    PredictionObstacles_writer->Write(local_view_.prediction_obstacles);
    Chassis_writer->Write(local_view_.chassis);
    RoutingResponse_writer->Write(local_view_.routing);
    i++;
    rate.Sleep();
  }

  //naruto add here=========================================
#if function_test1
    std::vector<double> x_out;
    std::vector<double> y_out;
    std::vector<double> x1_out;
    std::vector<double> y1_out;
    std::vector<double> routing_start_x;
    std::vector<double> routing_start_y;
    std::vector<double> routing_end_x;
    std::vector<double> routing_end_y;
    std::vector<double> vehicle_x;
    std::vector<double> vehicle_y;
    std::vector<double> vehicle_speed;
    std::vector<double> time_smaple;
    std::vector<double> vehicle_a;
    size_t trajectory_size = adc_trajectory_pb.trajectory_point_size();
    for(int i = 0; i < trajectory_size; i++) {
        x_out.push_back(adc_trajectory_pb.trajectory_point().at(i).path_point().x());
        y_out.push_back(adc_trajectory_pb.trajectory_point().at(i).path_point().y());
    }
    for(int i = 0; i < trajectory_size; i++) {
        //std::cout << adc_trajectory_pb.trajectory_point().at(i).relative_time() << std::endl;
        time_smaple.push_back(adc_trajectory_pb.trajectory_point().at(i).relative_time());
        vehicle_speed.push_back(adc_trajectory_pb.trajectory_point().at(i).v());
    }
    for(int i = 0; i < trajectory_size; i++) {
        //std::cout << adc_trajectory_pb.trajectory_point().at(i).relative_time() << std::endl;
        //time_smaple.push_back(adc_trajectory_pb.trajectory_point().at(i).relative_time());
        vehicle_a.push_back(adc_trajectory_pb.trajectory_point().at(i).a());
    }
    routing_start_x.push_back(586415.0);
    routing_start_y.push_back(4140270.0);
    routing_end_x.push_back(587169.10245);
    routing_end_y.push_back(4141539.780228);
    vehicle_x.push_back(586396.50656204869);
    vehicle_y.push_back(4140180.6837947061);
#if 0
    long fig1 = plt::figure(1);
    plt::clf();
    plt::axis("equal");
#if 1
    plt::subplot(2,2,1);
    plt::plot(x_out, y_out, "ro");
    plt::plot(vehicle_x, vehicle_y, "b*");
    plt::plot(routing_start_x, routing_start_y, "k*");
    plt::plot(routing_end_x, routing_end_y, "k*");
    plt::plot(x1_out, y1_out, "k*");
    plt::set_aspect_equal();
    plt::axis("equal");哔哩哔哩_bilibili
#endif
    plt::plot(time_smaple, vehicle_speed, "ro");
    plt::subplot(2,2,3);
    plt::plot(time_smaple, vehicle_a, "ro");
    plt::show();
#endif
#endif
  //=========================================================



  if (!IsValidTrajectory(adc_trajectory_pb)) {
    AERROR << "Fail to pass trajectory check.";
    return false;
  }

  adc_trajectory_ = adc_trajectory_pb;
  TrimPlanning(&adc_trajectory_, no_trajectory_point);
  if (FLAGS_test_update_golden_log) {
    AINFO << "The golden file is regenerated:" << full_golden_path;
    cyber::common::SetProtoToASCIIFile(adc_trajectory_, full_golden_path);
  } else {
    ADCTrajectory golden_result;
    bool load_success =
        cyber::common::GetProtoFromASCIIFile(full_golden_path, &golden_result);
    TrimPlanning(&golden_result, no_trajectory_point);
    if (!load_success ||
        !common::util::IsProtoEqual(golden_result, adc_trajectory_)) {
      char tmp_fname[100] = "/tmp/XXXXXX";
      int fd = mkstemp(tmp_fname);
      if (fd < 0) {
        AERROR << "Failed to create temporary file: " << tmp_fname;
        return false;
      }
      if (!cyber::common::SetProtoToASCIIFile(adc_trajectory_, fd)) {
        AERROR << "Failed to write to file: " << tmp_fname;
      }
      AERROR << "found error\ndiff -y " << tmp_fname << " " << full_golden_path;
      AERROR << "to override error\nmv " << tmp_fname << " "
             << full_golden_path;
      AERROR << "to visualize\n/usr/bin/python "
                "modules/tools/plot_trace/plot_planning_result.py "
             << tmp_fname << " " << full_golden_path;
      return false;
    }
  }



  return true;
}

bool PlanningTestBase::IsValidTrajectory(const ADCTrajectory& trajectory) {
  for (int i = 0; i < trajectory.trajectory_point_size(); ++i) {
    const auto& point = trajectory.trajectory_point(i);

    const double kMaxAccelThreshold =
        FLAGS_longitudinal_acceleration_upper_bound;
    const double kMinAccelThreshold =
        FLAGS_longitudinal_acceleration_lower_bound;
    if (point.a() > kMaxAccelThreshold || point.a() < kMinAccelThreshold) {
      AERROR << "Invalid trajectory point because accel out of range: "
             << point.DebugString();
      return false;
    }

    if (!point.has_path_point()) {
      AERROR << "Invalid trajectory point because NO path_point in "
                "trajectory_point: "
             << point.DebugString();
      return false;
    }

    if (i > 0) {
      const double kPathSEpsilon = 1e-3;
      const auto& last_point = trajectory.trajectory_point(i - 1);
      if (point.path_point().s() + kPathSEpsilon <
          last_point.path_point().s()) {
        AERROR << "Invalid trajectory point because s value error. last point: "
               << last_point.DebugString()
               << ", curr point: " << point.DebugString();
        return false;
      }
    }
  }
  return true;
}

//获取交通规则配置
TrafficRuleConfig* PlanningTestBase::GetTrafficRuleConfig(
    const TrafficRuleConfig::RuleId& rule_id) {
  for (auto& config : *planning_->traffic_rule_configs_.mutable_config()) {
    if (config.rule_id() == rule_id) {
      return &config;
    }
  }
  return nullptr;
}

}  // namespace planning
}  // namespace apollo

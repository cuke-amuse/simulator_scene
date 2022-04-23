#include <iostream>
#include "cyber/time/clock.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/common/local_view.h"
#include "modules/planning/common/ego_info.h"
#include "modules/planning/common/dependency_injector.h"
#include "modules/planning/planner/lattice/lattice_planner.h"
#include "modules/planning/integration_tests/planning_test_base_change.h"

//#include "modules/visualization/matplotlibcpp/matplotlibcpp.h"
//namespace plt = matplotlibcpp;
using apollo::cyber::Clock;
using namespace apollo::planning;

using namespace std;
int main(int argc, char* argv[]) 
{
    std::string input_file;
    std::string seq_num = "3"; 
    // init cyber framework
       char opt;
        while((opt = getopt(argc, argv, "hi:t:n:o:")) != -1) {
        switch(opt) {
            case 'h':
                printf("Usage: ./FileEnDeCryptor -i <InputFile> -t <E/D> -n <Num> -o <OutFile>\n");
                return 0;
            case 'i':
                input_file = string(optarg);
                break;
            case 't':
                if (!strcmp("E", optarg)) {
                    //option_type = TYPE_ENCRYPT;
                } else if (!strcmp("D", optarg)) {
                   // option_type = TYPE_DECRYPT;
                } else {
                    printf("Error: Option type must be E or D!\n");
                    return -1;
                }
                break;
            case 'n':
                //process_num = atoi(optarg);
                seq_num = string(optarg);
                break;
        }
    }

    apollo::cyber::Init("planning intergration test");
    PlanningTestBase test_base;
    test_base.SetUpTestCase();
    FLAGS_use_navigation_mode = false;
    FLAGS_map_dir = "/apollo/modules/map/data/sunnyvale_loop";
    FLAGS_test_base_map_filename = "base_map_test.bin";
    FLAGS_test_data_dir = "/apollo/modules/planning/testdata/" + input_file ;

    FLAGS_planning_upper_speed_limit = 12.5;
    FLAGS_use_multi_thread_to_add_obstacles = false;

    FLAGS_enable_scenario_stop_sign = false;
    FLAGS_enable_scenario_traffic_light = false;
    FLAGS_enable_rss_info = false;

    test_base.rule_enabled_[TrafficRuleConfig::CROSSWALK] = false;

    std::cout << "input file" << input_file << "/" << FLAGS_test_data_dir << "test seq:" << seq_num << std::endl;
    FLAGS_test_routing_response_file = seq_num + "_routing.pb.txt";
    FLAGS_test_prediction_file = seq_num + "_prediction.pb.txt";
    FLAGS_test_localization_file = seq_num + "_localization.pb.txt";
    FLAGS_test_chassis_file = seq_num + "_chassis.pb.txt";

    test_base.SetUp();
    bool no_trajectory_point = true;
    bool run_planning_success =
            test_base.RunPlanning("change_lane", 0, no_trajectory_point);
    std::cout << "run_planning_success resulst is: " << run_planning_success << std::endl;
    return 1;
}

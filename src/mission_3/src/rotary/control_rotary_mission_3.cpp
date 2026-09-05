/*
 * Mission 3 Control Rotary 노드의 진입점.
 * ROS2 초기화, CAV_IDS 기반 다중 CAV/HV Pub/Sub 설정, 그리고 메인 루프에서
 * Yellow Zone 시각화 -> ROI Pair 모니터링 -> Zone Group 판단 순서로
 * 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Rotary/Rotary.hpp"
#include "Visualizer/Visualizer.hpp"

// =========================
// CAV, HV Pose 콜백 함수들
// =========================
static void hv_19_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv_poses[19] = {msg->pose.position.x, msg->pose.position.y};
}

static void hv_20_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    hv_poses[20] = {msg->pose.position.x, msg->pose.position.y};
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_speed_trap_node");

    double detection_radius = 0.4;
    double resert_radius = 1.0;

    std::vector<std::pair<int, int>> roi_pairs = {
        {1, 1}, // CAV ROI 1 <-> HV ROI 1
        {2, 2}  // CAV ROI 2 <-> HV ROI 2
    };

    // Get TEAM_AIM_HOME environment variable (default: /root/TEAM_AIM)
    const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
    std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0)
                            ? std::string(team_aim_home)
                            : std::string("/root/TEAM_AIM");
    std::string path_dir = base_path + "/src/global_path/";

    int i = 19;
    std::string csv_path = path_dir + "path_mission3_" + std::to_string(i) + ".csv";
    hv_paths[i] = load_csv_file(csv_path);

    // Get CAV_IDS from environment variable (format: "32,3,5,6")
    const char* cav_ids_env = std::getenv("CAV_IDS");
    std::vector<int> active_cav_ids;
    if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
        std::string cav_ids_str(cav_ids_env);
        std::stringstream ss(cav_ids_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (!token.empty()) {
                active_cav_ids.push_back(std::stoi(token));
            }
        }
    } else {
        active_cav_ids = {1, 2, 3, 4};
    }

    // RED_FLAG & YELLOW_FLAG Publisher (based on CAV_IDS indices 1-4)
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> yellow_flag_pubs;
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> target_vel_pubs;

    // Marker Publisher for yellow zone visualization
    auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/yellow_zones_markers", 10);

    for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
        int cav_index = (int)i + 1;  // 1-indexed
        int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
        const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
        const std::string flag_topic = "/CAV_" + cav_id_str + "_RED_FLAG";
        red_flag_pubs[cav_index] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 1);
        const std::string yellow_flag_topic = "/CAV_" + cav_id_str + "_YELLOW_FLAG";
        yellow_flag_pubs[cav_index] = node->create_publisher<std_msgs::msg::Int32>(yellow_flag_topic, 1);
        const std::string vel_topic = "/CAV_" + cav_id_str + "_target_vel";
        target_vel_pubs[cav_index] = node->create_publisher<std_msgs::msg::Float64>(vel_topic, 1);
    }

    // Subscribers for CAV indices (1-4), HV 19, 20
    std::vector<std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>>> cav_subscriptions;

    for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
        int cav_index = (int)i + 1;  // 1-indexed
        int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
        const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
        std::string cav_topic = "/CAV_" + cav_id_str;
        auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            cav_topic, rclcpp::SensorDataQoS().keep_last(15),
            [cav_index](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                cav_poses[cav_index] = {msg->pose.position.x, msg->pose.position.y};
            });
        cav_subscriptions.push_back(sub);
    }

    auto sub19 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_19", rclcpp::SensorDataQoS().keep_last(15),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          hv_19_callback(msg);
    });

    auto sub20 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_20", rclcpp::SensorDataQoS().keep_last(15),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          hv_20_callback(msg);
    });

    auto sub_hv19 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_19", rclcpp::SensorDataQoS().keep_last(15),
        [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            hv_19_callback(msg);
            CalculateInstantVelocity(msg, 19, hv_states);
    });



    rclcpp::Rate r(50);
    std::map<int, std::vector<int>> prev_red_flag_vehicles;
    std::map<int, std::set<int>> cav_active_states;
    std::map<int, std::set<int>> cav_released_states;

    double yellow_roi_detection_radius = 0.5;  // yellow ROI 감지 반경 0.3 / 0.5 / 0.4

    while(rclcpp::ok()) {
        // Publish yellow zone markers for visualization in RViz
        publish_yellow_zone_markers(node, marker_pub, yellow_roi_detection_radius);

        RotaryProcess(roi_pairs, node, red_flag_pubs, yellow_flag_pubs, target_vel_pubs,
                      detection_radius, resert_radius, yellow_roi_detection_radius);

        rclcpp::spin_some(node);
        r.sleep();
    }
    rclcpp::shutdown();
    return 0;
}

/*
 * Mission 3 Control Tower 노드의 진입점.
 * ROS2 초기화, CAV_IDS 기반 다중 CAV/HV Pub/Sub 설정, 그리고 메인 루프에서
 * Zone 모니터링 -> 시각화 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Tower/Tower.hpp"
#include "Visualizer/Visualizer.hpp"

// =========================
// HV Pose 콜백 함수들
// =========================
static void hv_19_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hv_poses[19] = {msg->pose.position.x, msg->pose.position.y};
}

static void hv_20_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  hv_poses[20] = {msg->pose.position.x, msg->pose.position.y};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control_tower");

  double precollision_radius = 2.3;  // ** 2.3 -> 2.5
  double imminent_collision_radius = 1.8; // ** 1.8 -> 2.0
  double overlap_threshold = 0.2; // ** 0.2 -> 0.5 -> 0.7
  int lookahead_distance = 90;  // 경로 겹침 체크용 lookahead 거리
  int visualization_lookahead = 90;  // 시각화용 lookahead 거리

  // Get TEAM_AIM_HOME environment variable (default: /root/TEAM_AIM)
  const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
  std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0)
                          ? std::string(team_aim_home)
                          : std::string("/root/TEAM_AIM");
  std::string path_dir = base_path + "/src/global_path/";

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

  for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
      int cav_index = (int)i + 1;  // 1-indexed
      std::string cav_index_str = std::string(2 - std::to_string(cav_index).length(), '0') + std::to_string(cav_index);
      std::string csv_path = path_dir + "path_mission3_" + cav_index_str + ".csv";
      cav_paths[cav_index] = load_csv_file(csv_path);
  }

  // RED_FLAG Publisher (based on active CAV count)
  std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
  std::map<int, int> cav_index_map_actual_to_idx;  // Map actual_cav_id to cav_index
  std::map<int, int> cav_index_map_idx_to_actual;  // Map cav_index to actual_cav_id (reverse)
  for (size_t i = 0; i < active_cav_ids.size() && i < 4; i++) {
      int cav_index = (int)i + 1;  // 1-indexed
      int actual_cav_id = active_cav_ids[i];  // actual CAV ID from CAV_IDS
      cav_index_map_actual_to_idx[actual_cav_id] = cav_index;  // Store mapping for CSV lookup
      cav_index_map_idx_to_actual[cav_index] = actual_cav_id;  // Store reverse mapping for RED_FLAG publishing
      const std::string cav_id_str = std::string(2 - std::to_string(actual_cav_id).length(), '0') + std::to_string(actual_cav_id);
      const std::string flag_topic = "/CAV_" + cav_id_str + "_RED_FLAG";
      red_flag_pubs[actual_cav_id] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 1);
  }

  //  Visualization Publisher 추가
  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/control_tower/visualization", 10);

  // Subscribers for CAV indices (1-4)
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

  rclcpp::Rate r(50);
  std::map<int, std::vector<int>> prev_red_flag_vehicles;

  while (rclcpp::ok()) {
    //  Zone 1, 2, 3만 모니터링 (Zone 4 제거)
    for (int zone_id = 1; zone_id <= 3; zone_id++) {
      monitor_zone(zone_id, node, red_flag_pubs, prev_red_flag_vehicles,
                   precollision_radius, imminent_collision_radius, overlap_threshold, lookahead_distance, cav_index_map_actual_to_idx, cav_index_map_idx_to_actual);
    }
    // 시각화 발행 (10Hz)
    publish_visualization(node, marker_pub, precollision_radius, imminent_collision_radius, visualization_lookahead);

    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

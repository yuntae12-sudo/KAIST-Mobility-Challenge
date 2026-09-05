/*
 * Mission 1-2 Control Tower 노드의 진입점.
 * ROS2 초기화, Pub/Sub 설정, 그리고 메인 루프에서 ROI 기반 CAV 제어 판단 ->
 * Zone 모니터링 -> 시각화 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Tower/Tower.hpp"
#include "Visualizer/Visualizer.hpp"

// =========================
// CAV Pose 콜백 함수들
// =========================
static void cav_01_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[1] = {msg->pose.position.x, msg->pose.position.y};
}

static void cav_02_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    cav_poses[2] = {msg->pose.position.x, msg->pose.position.y};
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control_tower");

  double precollision_radius = 2.3;
  double imminent_collision_radius = 1.6;
  double overlap_threshold = 0.2;
  int lookahead_distance = 100;
  int visualization_lookahead = 20;
  std::string path_dir = "/root/TEAM_AIM/src/global_path/";

  RCLCPP_INFO(node->get_logger(), "Loading CSV path files...");
  for (int i = 1; i <= 2; i++) {
    std::string csv_path = path_dir + "path_mission1_0" + std::to_string(i) + ".csv";
    cav_paths[i] = load_csv_file(csv_path);
    RCLCPP_INFO(node->get_logger(), "CAV_%d: %zu waypoints loaded", i, cav_paths[i].size());
  }

  std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> red_flag_pubs;
  for (int i = 1; i <= 2; i++) {
    const std::string flag_topic = "/CAV_0" + std::to_string(i) + "_RED_FLAG";
    red_flag_pubs[i] = node->create_publisher<std_msgs::msg::Int32>(flag_topic, 50);
  }

  auto marker_pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/control_tower/visualization", 10);

  auto sub1 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_01", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_01_callback(msg);
      });

  auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/CAV_02", rclcpp::SensorDataQoS(),
      [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        cav_02_callback(msg);
      });


  rclcpp::Rate r(50);
  std::map<int, std::vector<int>> prev_red_flag_vehicles;

  RCLCPP_INFO(node->get_logger(), "Control Tower Node started! (Zone 2,4 + ROI 1,2,3)");

  while (rclcpp::ok()) {
    // ROI 기반 CAV2 제어 (실시간 위치 기반)
    bool cav2_should_stop = should_cav2_stop_by_roi(node);

    static bool prev_cav2_roi_stop = false;
    if (cav2_should_stop != prev_cav2_roi_stop) {
      auto msg = std_msgs::msg::Int32();
      msg.data = cav2_should_stop ? 1 : 0;
      red_flag_pubs[2]->publish(msg);

      if (cav2_should_stop) {
        RCLCPP_ERROR(node->get_logger(), "[ROI CONTROL] CAV_2 RED_FLAG by ROI logic");
      } else {
        RCLCPP_INFO(node->get_logger(), "[ROI CONTROL] CAV_2 GREEN_FLAG by ROI logic");
      }

      prev_cav2_roi_stop = cav2_should_stop;
    }

    // ---------------------------------------------------------
    // ROI 4 기반 CAV1 제어 (합류 구간)
    // ---------------------------------------------------------
    bool cav1_merge_stop = should_cav1_stop_by_merge_roi(node);
    static bool prev_cav1_merge_stop = false;

    // 상태가 변했을 때만 Publish (토픽 부하 방지)
    if (cav1_merge_stop != prev_cav1_merge_stop) {
        auto msg = std_msgs::msg::Int32();
        msg.data = cav1_merge_stop ? 1 : 0;
        red_flag_pubs[1]->publish(msg); // CAV1에게 정지 신호 보냄

        if (cav1_merge_stop) {
            RCLCPP_ERROR(node->get_logger(), "[MERGE CONTROL] CAV_1 RED_FLAG! (Conflict at ROI 4)");
        } else {
            RCLCPP_INFO(node->get_logger(), "[MERGE CONTROL] CAV_1 GREEN_FLAG (ROI 4 Clear)");
        }

        prev_cav1_merge_stop = cav1_merge_stop;
    }

    // Zone 2, 4만 모니터링
    monitor_zone(2, node, red_flag_pubs, prev_red_flag_vehicles,
                 precollision_radius, imminent_collision_radius,
                 overlap_threshold, lookahead_distance);
    monitor_zone(4, node, red_flag_pubs, prev_red_flag_vehicles,
                 precollision_radius, imminent_collision_radius,
                 overlap_threshold, lookahead_distance);

    publish_visualization(node, marker_pub, precollision_radius,
                         imminent_collision_radius, visualization_lookahead);

    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

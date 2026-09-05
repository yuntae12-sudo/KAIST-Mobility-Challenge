/*
 * Mission 3 CAV 제어 노드의 진입점.
 * ROS2 초기화, CAV_IDS 기반 다중 CAV Pub/Sub 설정, 그리고 Pose 수신마다
 * Mission -> Planning -> Control 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Planning/Planning.hpp"
#include "Control/Control.hpp"
#include "Mission/Mission.hpp"

static std::vector<integrate_path_struct> integrate_path_vector;

// Global mission completion flag
static bool mission_completed = false;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control");
  auto st   = std::make_shared<ControllerState>();

  int total_vehicle_count = 32;
  std::vector<CavState> cav_list(total_vehicle_count + 1);
  for(int i=1; i<(int)cav_list.size(); ++i) cav_list[i] = {i, 0.0, 0.0, 0, false, false, false, rclcpp::Time()};

  // 실제 참여 차량 수
  int actual_vehicle_count = 0;

  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int actual_cav_id = node->get_parameter("cav_id").as_int();

  if(actual_cav_id >= (int)cav_list.size()) {
      RCLCPP_ERROR(node->get_logger(), "CAV_ID(%d) Error", actual_cav_id);
      return -1;
  }

  // Get CAV_IDS from environment variable to determine cav_index (1-4)
  const char* cav_ids_env = std::getenv("CAV_IDS");
  int cav_index = 1;  // Default to 1
  if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
      std::string cav_ids_str(cav_ids_env);
      std::stringstream ss(cav_ids_str);
      std::string token;
      int idx = 1;
      while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
              actual_vehicle_count++;  // 실제 참여 차량 수 증가
              int id = std::stoi(token);
              if (id == actual_cav_id) {
                  cav_index = idx;
                  break;
              }
              idx++;
          }
      }
  }
// sudo chmod 666 /dev/ttyUSB0   # 임시(재부팅/재연결 시 원복될 수 있음)
// ./cav_4_SDK.sh

  const std::string my_id_str = twoDigitId(actual_cav_id);  // Use actual_cav_id for all topics
  const std::string accel_topic = "/CAV_" + my_id_str + "_accel";
  const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string yellow_flag_topic = "/CAV_" + my_id_str + "_YELLOW_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";
  const std::string cmd_vel_topic = "/CAV_" + my_id_str + "/cmd_vel_"; // motor_topic

  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 5.5); // [핵심] 고속 주행을 위해 Yaw Rate 제한 대폭 해제 // ** 5.5 -> 2.5 ?** // ****
  node->declare_parameter<std::string>("path_csv", "/home/aim/TEAM_AIM/src/global_path/path.csv");

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  // Get TEAM_AIM_HOME environment variable (default: /home/aim/TEAM_AIM)
  const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
  std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0)
                          ? std::string(team_aim_home)
                          : std::string("/home/aim/TEAM_AIM");
  const std::string path_with_id_csv = base_path + "/src/global_path/" + "path_mission3_" + std::string(2 - std::to_string(cav_index).length(), '0') + std::to_string(cav_index) + ".csv";
  if (!loadPathCsv(path_with_id_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_with_id_csv.c_str());
    rclcpp::shutdown(); return 1;
  }

  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 1, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; });

  auto yellow_flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      yellow_flag_topic, 1, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->yellow_flag = msg->data; });

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 1,
      [st](const std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data < 0.0) {
              st->tower_mode = false;
          } else {
              st->tower_mode = true;
              st->target_linear_x = msg->data;
          }
      }
  );

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs;

  // Get actual CAV IDs from environment variable (reuse existing cav_ids_env)
  std::vector<int> all_active_cav_ids;
  if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
      std::string cav_ids_str(cav_ids_env);
      std::stringstream ss(cav_ids_str);
      std::string token;
      while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
              all_active_cav_ids.push_back(std::stoi(token));
          }
      }
  }

  // Subscribe to ALL CAV topics while maintaining CSV index mapping
  for (size_t i = 0; i < all_active_cav_ids.size(); ++i) {
      int csv_index = (int)i + 1;  // 1, 2, 3, 4 (for CSV path mapping)
      int actual_subscribe_cav_id = all_active_cav_ids[i];  // actual CAV ID to subscribe

      std::string target_topic = "/CAV_" + twoDigitId(actual_subscribe_cav_id);

      auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::SensorDataQoS().keep_last(15),
          [node, st, accel_pub, cmd_vel_pub, &cav_list, actual_cav_id, csv_index, cav_index, actual_vehicle_count, cmd_vel_topic](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            // 1) Mission: Lap 상태 갱신 및 시작 정지 활성화 (모든 CAV 대상)
            MissionProcess(msg, csv_index, cav_list, cav_index, st);

            // Check mission completion and set global flag
            if (CheckAllFinished(cav_list, actual_vehicle_count) && !mission_completed) {
                mission_completed = true;
            }

            // Execute control ONLY for my own CAV
            if (csv_index == cav_index) {
                // 미션 완료 후 계속 정지 명령 발행 (실제 로봇이 멈추도록)
                if (mission_completed) {
                    geometry_msgs::msg::Accel stop_cmd;
                    stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
                    stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
                    accel_pub->publish(stop_cmd);

                    geometry_msgs::msg::Twist stop_twist;
                    // fix : linear.x 0 -> -0.005
                    stop_twist.linear.x = -0.005; stop_twist.linear.y = 0.0; stop_twist.linear.z = 0.0;
                    stop_twist.angular.x = 0.0; stop_twist.angular.y = 0.0; stop_twist.angular.z = 0.0;
                    cmd_vel_pub->publish(stop_twist);
                    return;
                }

                get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);
                double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
                if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
                    yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
                }
                st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;

                // 2) Planning: Closest Point / Corner 판단 (이전 주기 lookahead_m 기준)
                int closest_idx = 0;
                bool corner_detected = false;
                PlanningProcess(integrate_path_vector, x_m, y_m, *st, closest_idx, corner_detected);

                // 3) Control: 목표 속도 계획, 속도 램프, (Planning) 목표 Waypoint 재탐색,
                //    Pure Pursuit, 우선순위 기반 명령 생성까지 순서대로 수행한다.
                geometry_msgs::msg::Accel cmd;
                geometry_msgs::msg::Twist twist_cmd;
                ControlProcess(*st, corner_detected, integrate_path_vector,
                               x_m, y_m, yaw, msg->header.stamp, mission_completed, actual_cav_id,
                               cmd, twist_cmd);

                accel_pub->publish(cmd);    // simulator
                cmd_vel_pub->publish(twist_cmd); // motor
            }
          }
      );
      pose_subs.push_back(sub);

  }

  (void)flag_sub; (void)yellow_flag_sub; (void)vel_sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

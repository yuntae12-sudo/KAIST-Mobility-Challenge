/*
 * Mission 1-2 CAV 제어 노드의 진입점.
 * ROS2 초기화, Pub/Sub 설정, 그리고 Pose 수신마다 Mission -> Planning -> Control
 * 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp> // [수정] Float64 사용

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Planning/Planning.hpp"
#include "Control/Control.hpp"
#include "Mission/Mission.hpp"

static std::vector<integrate_path_struct> integrate_path_vector;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control");
  auto st   = std::make_shared<ControllerState>();

  int total_vehicle_count = 2;
  std::vector<CavState> cav_list(total_vehicle_count + 1);
  for(int i=1; i<(int)cav_list.size(); ++i) cav_list[i] = {i, 0.0, 0.0, 0, false, false, false};

  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int cav_id = node->get_parameter("cav_id").as_int();

  if(cav_id >= (int)cav_list.size()) {
      RCLCPP_ERROR(node->get_logger(), "CAV_ID(%d) Error", cav_id);
      return -1;
  }

  const std::string my_id_str = twoDigitId(cav_id);
  const std::string accel_topic = "/CAV_" + my_id_str + "_accel";
  const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";

  RCLCPP_INFO(node->get_logger(), "My CAV_ID=%d, Waiting for %d vehicles...", cav_id, total_vehicle_count);

  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 4.0); // [핵심] 고속 주행을 위해 Yaw Rate 제한 대폭 해제
  node->declare_parameter<std::string>("path_csv", "/root/TEAM_AIM/src/global_path/path_mission1_01.csv");

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  std::string path_with_id_csv;
  if (my_id_str == "01") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_01.csv";
  } else if (my_id_str == "02") {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_02.csv";
  } else {
    path_with_id_csv = "/root/TEAM_AIM/src/global_path/path_mission1_01.csv";
  }
  if (!loadPathCsv(path_with_id_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_with_id_csv.c_str());
    rclcpp::shutdown(); return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded path: %zu waypoints from %s", integrate_path_vector.size(), path_with_id_csv.c_str());

  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 10, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; });

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 10,
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

  for (int target_id = 1; target_id < total_vehicle_count + 1; ++target_id) {
      std::string target_topic = "/CAV_" + twoDigitId(target_id);
      auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::SensorDataQoS(),
          [node, st, accel_pub, &cav_list, cav_id, target_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            // 1) Mission: Lap 상태 갱신 (모든 CAV 대상)
            MissionProcess(msg, target_id, cav_list);

            if (target_id == cav_id) {
                if (CheckAllFinished(cav_list)) {
                    geometry_msgs::msg::Accel stop_cmd;
                    stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
                    stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
                    accel_pub->publish(stop_cmd);
                    static int stop_log_cnt = 0;
                    if (stop_log_cnt++ % 50 == 0) std::cout << ">>> [ALL FINISHED] SYNC STOP! <<<" << std::endl;
                    return;
                }

                get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);
                double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
                if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
                    yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
                }
                st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;

                // 2) Planning: Closest Point / Corner 판단
                int closest_idx = 0;
                bool corner_detected = false;
                PlanningProcess(integrate_path_vector, x_m, y_m, *st, closest_idx, corner_detected);

                // 3) Control: 목표 속도 결정
                double current_target_speed = DecideTargetSpeed(*st, corner_detected);

                // 4) Planning: 목표 속도 반영 후 목표 Waypoint 탐색
                int target_path_idx = FindTargetWaypoint(integrate_path_vector, x_m, y_m, *st);

                // 5) Control: Pure Pursuit 각속도 계산 및 명령 생성
                geometry_msgs::msg::Accel cmd;
                ControlProcess(*st, current_target_speed, integrate_path_vector, target_path_idx,
                               x_m, y_m, yaw, cmd);

                accel_pub->publish(cmd);
            }
          }
      );
      pose_subs.push_back(sub);
      RCLCPP_INFO(node->get_logger(), "Subscribed to %s", target_topic.c_str());
  }

  (void)flag_sub; (void)vel_sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

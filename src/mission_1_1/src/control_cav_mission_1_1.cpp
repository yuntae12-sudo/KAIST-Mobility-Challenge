/*
 * Mission 1-1 CAV 제어 노드의 진입점.
 * ROS2 초기화, Pub/Sub 설정, 그리고 Pose 수신마다 Mission -> Planning -> Control
 * 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
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

  // ---- CAV ID
  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int cav_id = node->get_parameter("cav_id").as_int();
  const std::string my_id_str = twoDigitId(cav_id);

  // ---- Topics
  const std::string my_pose_topic    = "/CAV_" + my_id_str;               // 내 pose만 구독
  const std::string accel_topic      = "/CAV_" + my_id_str + "_accel";
  const std::string red_flag_topic   = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";
  // const std::string cmd_vel_topic    = "/CAV_" + my_id_str + "/cmd_vel";
  const std::string cmd_vel_topic    = "/cmd_vel";

  RCLCPP_INFO(node->get_logger(), "My CAV_ID=%d", cav_id);

  // ---- Params
  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 5.5);

  // [핵심] 경로를 mission1_01로 고정
  // Get TEAM_AIM_HOME environment variable (default: /home/aim/TEAM_AIM)
  const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
  std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0)
                          ? std::string(team_aim_home)
                          : std::string("/home/aim/TEAM_AIM");
  node->declare_parameter<std::string>(
      "path_csv",
      base_path + "/src/global_path/path_mission1_01.csv"
  );

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  const std::string path_csv = node->get_parameter("path_csv").as_string();
  if (!loadPathCsv(path_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_csv.c_str());
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Loaded path: %s (%zu waypoints)", path_csv.c_str(), integrate_path_vector.size());

  // ---- Lap state (내 차량만)
  CavState my_cav {cav_id, 0.0, 0.0, 0, false, false, false};

  // ---- Pub/Sub
  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 50,
      [st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; }
  );

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 50,
      [st](const std_msgs::msg::Float64::SharedPtr msg) {
        if (msg->data < 0.0) {
          st->tower_mode = false;
        } else {
          st->tower_mode = true;
          st->target_linear_x = msg->data;
        }
      }
  );

  // ---- 내 차량 pose만 구독
  auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
      my_pose_topic, rclcpp::SensorDataQoS(),
      [st, accel_pub, cmd_vel_pub, &my_cav](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {
        // 1) Mission: Lap/Finish 업데이트 (내 차량만)
        MissionProcess(msg, my_cav);

        // 완주 후엔 계속 정지 명령 유지
        if (my_cav.finished) {
          geometry_msgs::msg::Accel stop_cmd;
          stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
          stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
          accel_pub->publish(stop_cmd);
          geometry_msgs::msg::Twist stop_twist;
          stop_twist.linear.x = 0.0; stop_twist.linear.y = 0.0; stop_twist.linear.z = 0.0;
          stop_twist.angular.x = 0.0; stop_twist.angular.y = 0.0; stop_twist.angular.z = 0.0;
          cmd_vel_pub->publish(stop_twist);
          return;
        }

        // 2) pose/yaw
        get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);

        double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
        if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
          yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
        }
        st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;

        // 3) Planning: Closest Point / Corner 판단
        int closest_idx = 0;
        bool corner_detected = false;
        PlanningProcess(integrate_path_vector, x_m, y_m, *st, closest_idx, corner_detected);

        // 4) Control: 목표 속도 결정
        double current_target_speed = DecideTargetSpeed(*st, corner_detected);

        // 5) Planning: 목표 속도 반영 후 목표 Waypoint 탐색
        int target_path_idx = FindTargetWaypoint(integrate_path_vector, x_m, y_m, *st);
        if (target_path_idx < 0) return;

        // 6) Control: Pure Pursuit 각속도 계산 및 명령 생성
        geometry_msgs::msg::Accel cmd;
        geometry_msgs::msg::Twist twist_cmd;
        ControlProcess(*st, current_target_speed, integrate_path_vector, target_path_idx,
                       x_m, y_m, yaw, cmd, twist_cmd);

        accel_pub->publish(cmd);
        cmd_vel_pub->publish(twist_cmd);
      }
  );

  RCLCPP_INFO(node->get_logger(), "Subscribed ONLY to %s", my_pose_topic.c_str());

  (void)pose_sub; (void)flag_sub; (void)vel_sub;

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

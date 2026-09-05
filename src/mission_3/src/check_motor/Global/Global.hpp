/*
 * check_motor 노드 전체에서 공유되는 자료구조와 위치(Localization) 전역 상태를 정의한다.
 * Planning/Control/Mission 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_3_CHECK_MOTOR_GLOBAL_HPP
#define MISSION_3_CHECK_MOTOR_GLOBAL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

// =========================
// Path data
// =========================
struct integrate_path_struct {
  double x;
  double y;
};

// =========================
// Controller state
// =========================
struct ControllerState
{
  double speed_mps{0.5};
  double lookahead_m{0.3};
  double max_yaw_rate{2.0}; // 5.5는 센서 노이즈로 오차가 있어 2.0으로 낮춤

  double prev_x{0.0};
  double prev_y{0.0};
  double prev_yaw{0.0};
  bool has_prev{false};

  int pos_count{0};
  int red_flag{0};
  int yellow_flag{0};  // Yellow flag for speed control
  bool tower_mode{false};
  double target_linear_x{0.0};
};

// =========================
// CAV Structures
// =========================
struct CavState {
    int id;
    double start_x;
    double start_y;
    int current_lap;
    bool is_initialized;
    bool is_in_zone;
    bool finished;
    rclcpp::Time lap_start_time;  // Lap start time for lap timing
};

// =========================
// Localization globals
// =========================
extern double x_m, y_m, z_m;
extern double x_q, y_q, z_q, w_q;
extern int closest_index;

void get_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
              double &x_m, double &y_m, double &z_m,
              double &x_q, double &y_q, double &z_q, double &w_q);

#endif  // MISSION_3_CHECK_MOTOR_GLOBAL_HPP

/*
 * Mission 3 CAV 제어 노드 전체에서 공유되는 자료구조와 위치(Localization) 전역 상태를 정의한다.
 * Planning/Control/Mission 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_3_CAV_GLOBAL_HPP
#define MISSION_3_CAV_GLOBAL_HPP

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

  // Velocity ramp control
  double current_velocity{0.0};  // 실제 발행되는 속도
  double max_acceleration{2.5};  // 최대 가속도 (m/s²) - 조정 가능

  // Red flag 상태 추적 (정지 후 느리게 속도 올림)
  int red_flag_hold_count{0};  // Red flag 지속 카운트
  const int RED_FLAG_RELEASE_STABILIZE_CYCLES = 100;  // ~1초 (50Hz에서)
  double max_acceleration_after_red_flag{2.5};  // Red flag 해제 후 느린 가속도

  // 시작 3초 정지
  rclcpp::Time start_stop_time;  // 시작 정지 시작 시간
  bool is_start_stop_active{false};  // 정지 중인지 여부
  const int START_STOP_DURATION_MS = 4000;  // 3초

  rclcpp::Time last_update_time;  // 마지막 업데이트 시간
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

#endif  // MISSION_3_CAV_GLOBAL_HPP

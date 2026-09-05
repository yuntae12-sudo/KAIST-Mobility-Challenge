/*
 * Control.hpp에 선언된 목표 속도 계획, Pure Pursuit, 제어 명령 생성 함수들의 구현.
 */
#include "Control/Control.hpp"

#include <algorithm>
#include <cmath>

#include "Planning/Planning.hpp"

// 목표 속도 결정, (Planning) 목표 Waypoint 재탐색, Pure Pursuit 각속도 계산,
// 최종 명령 생성을 순서대로 수행한다.
void ControlProcess(ControllerState& st,
                     bool corner_detected,
                     const std::vector<integrate_path_struct>& path,
                     double x_m, double y_m, double yaw,
                     bool mission_completed,
                     geometry_msgs::msg::Accel& cmd,
                     geometry_msgs::msg::Twist& twist_cmd)
{
  double current_target_speed;
  if (st.tower_mode) {
    current_target_speed = st.target_linear_x;
  } else {
    planVelocity(st, corner_detected);
    current_target_speed = st.speed_mps;
  }

  st.speed_mps = current_target_speed;

  // 목표 속도가 반영된 Lookahead로 목표 Waypoint를 다시 탐색한다.
  int target_path_idx = FindTargetWaypoint(path, x_m, y_m, st);

  double wz = PurePursuitAngularVelocity(path, target_path_idx, x_m, y_m, yaw,
                                          current_target_speed, st.max_yaw_rate);

  FillControlCommand(st, current_target_speed, wz, mission_completed, cmd, twist_cmd);
}

// 직선(2.0)/커브(1.5) 구간에 따라 목표 속도를 정한다.
void planVelocity(ControllerState& st, bool isCorner) {
    if (!isCorner) {st.speed_mps = 2.0; } else { st.speed_mps = 1.5; } // good (1.5 / 1.0) -> (1.5 / 0.8)
    // if (!isCorner) {st.speed_mps = 1.7; } else { st.speed_mps = 1.4; } // good (1.5 / 1.0) -> (1.5 / 0.8)
    /*
    ver1 -> 1.5     / 1.0     / 0.8
    ver2 -> 1.6     / 1.2     / 1.0
    ver3 -> 1.6     / 1.2     / 1.1
    ver4 -> 1.7     / 1.3     / 0.8
    ver5 -> 1.75    / 1.3     / 0.8
    ver 6 -> 1.75    / 1.3     / 1.0
    ver 7 -> 1.8     / 1.5     / 0.8
    ver 8 -> 1.8    / 1.5     / 1.0
    ver 9 -> 1.9     / 1.5     / 0.8
    ver 10 - > 1.9    /   1.5.  /   1.0
    ver 11 -> 2.0     / 1.5     / 0.8
    ver 12 -> 2.0     / 1.5     / 1.0
    */
}

// 목표 Waypoint와 현재 위치/자세로부터 곡률을 구하고, 이를 각속도로 변환한다.
double PurePursuitAngularVelocity(const std::vector<integrate_path_struct>& path,
                                   int target_path_idx,
                                   double x_m, double y_m, double yaw,
                                   double speed, double max_yaw_rate)
{
  const double dx = path[target_path_idx].x - x_m;
  const double dy = path[target_path_idx].y - y_m;

  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double x_v =  cy * dx + sy * dy;
  const double y_v = -sy * dx + cy * dy;
  const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
  const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);

  double wz = speed * kappa;
  wz = std::clamp(wz, -max_yaw_rate, max_yaw_rate);
  return wz;
}

// 완주 -> Red Flag -> Yellow Flag -> 일반 주행 순서의 우선순위로 명령을 채운다.
void FillControlCommand(const ControllerState& st,
                         double target_speed, double wz,
                         bool mission_completed,
                         geometry_msgs::msg::Accel& cmd,
                         geometry_msgs::msg::Twist& twist_cmd)
{
  // Check if ALL CAVs have finished 5 laps
  if (mission_completed) {
      // All CAVs finished 5 laps - STOP
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      twist_cmd.linear.x = -0.005; // Stop command
      twist_cmd.angular.z = 0.0;
  } else if (st.red_flag == 1) {
      // Red flag: STOP with -0.005
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      twist_cmd.linear.x = -0.005; // Stop command
      twist_cmd.angular.z = 0.0;
  } else if (st.yellow_flag == 1) {
      // Yellow flag: Slow down to 0.8 m/s
      cmd.linear.x  = 0.8; // 0.8 -> 1.0
      cmd.angular.z = wz;
      twist_cmd.linear.x = 0.8; // 0.8 -> 1.5
      twist_cmd.angular.z = wz;
  } else {
      cmd.linear.x  = target_speed;
      cmd.angular.z = wz;
      twist_cmd.linear.x = target_speed;
      twist_cmd.angular.z = wz;
  }
  cmd.linear.y = 0.0; cmd.linear.z = 0.0;
  cmd.angular.x = 0.0; cmd.angular.y = 0.0;

  twist_cmd.linear.y = 0.0; twist_cmd.linear.z = 0.0;
  twist_cmd.angular.x = 0.0; twist_cmd.angular.y = 0.0;
}

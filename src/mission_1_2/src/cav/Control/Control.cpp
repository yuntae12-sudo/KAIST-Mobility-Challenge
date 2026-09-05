/*
 * Control.hpp에 선언된 목표 속도 계획, Pure Pursuit, 제어 명령 생성 함수들의 구현.
 */
#include "Control/Control.hpp"

#include <algorithm>
#include <cmath>

// 타워 모드면 타워가 지정한 속도를, 아니면 커브 여부에 따른 계획 속도를 목표 속도로 반영한다.
double DecideTargetSpeed(ControllerState& st, bool corner_detected)
{
  double current_target_speed;
  if (st.tower_mode) {
    current_target_speed = st.target_linear_x;
  } else {
    planVelocity(st, corner_detected);
    current_target_speed = st.speed_mps;
  }

  // [중요] 타워 모드일 때 속도가 빨라지면 Ld가 너무 커지는 것을 막기 위해
  // GetLd 내부에서 max_ld를 0.35로 꽉 잡고 있습니다.
  st.speed_mps = current_target_speed;
  return current_target_speed;
}

// 목표 Waypoint를 향한 각속도를 계산하고, Red Flag 상태에 따라 최종 명령을 채운다.
void ControlProcess(const ControllerState& st,
                     double target_speed,
                     const std::vector<integrate_path_struct>& path,
                     int target_path_idx,
                     double x_m, double y_m, double yaw,
                     geometry_msgs::msg::Accel& cmd)
{
  double wz = PurePursuitAngularVelocity(path, target_path_idx, x_m, y_m, yaw,
                                          target_speed, st.max_yaw_rate);
  FillControlCommand(st, target_speed, wz, cmd);
}

// 직선/커브 구간에 따라 목표 속도를 정한다.
void planVelocity(ControllerState& st, bool isCorner) {
  if (!isCorner) { st.speed_mps = 2.0; } else { st.speed_mps = 1.5; }
}

// 목표 Waypoint와 현재 위치/자세로부터 곡률을 구하고, 이를 각속도로 변환한다.
double PurePursuitAngularVelocity(const std::vector<integrate_path_struct>& path,
                                   int target_path_idx,
                                   double x_m, double y_m, double yaw,
                                   double target_speed, double max_yaw_rate)
{
  const double dx = path[target_path_idx].x - x_m;
  const double dy = path[target_path_idx].y - y_m;

  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double x_v =  cy * dx + sy * dy;
  const double y_v = -sy * dx + cy * dy;
  const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
  const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);

  double wz = target_speed * kappa;
  wz = std::clamp(wz, -max_yaw_rate, max_yaw_rate);
  return wz;
}

// Red Flag가 설정되어 있으면 정지 명령을, 아니면 계산된 속도/각속도 명령을 채운다.
void FillControlCommand(const ControllerState& st,
                         double target_speed, double wz,
                         geometry_msgs::msg::Accel& cmd)
{
  if (st.red_flag == 1) {
    cmd.linear.x  = 0.0;
    cmd.angular.z = 0.0;
  } else {
    cmd.linear.x  = target_speed;
    cmd.angular.z = wz;
  }
  cmd.linear.y = 0.0; cmd.linear.z = 0.0;
  cmd.angular.x = 0.0; cmd.angular.y = 0.0;
}

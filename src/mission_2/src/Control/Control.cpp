/*
 * Control.hpp에 선언된 목표 속도 계획, Pure Pursuit, 제어 명령 생성 함수들의 구현.
 */
#include "Control/Control.hpp"

#include <algorithm>
#include <cmath>

#include "Planning/Planning.hpp"

// Closest Point/커브 판단 후 목표 속도를 계획하고, Pure Pursuit 각속도를 계산하여 최종 명령을 채운다.
bool ControlProcess(ControllerState& st,
                     bool stop_flag, bool race_over,
                     std::shared_ptr<rclcpp::Node> node,
                     geometry_msgs::msg::Accel& cmd)
{
  // ===== 10. Pure Pursuit =====
  int closest_idx = findClosestPoint(integrate_path_vector, cav_x, cav_y, current_lane);

  if (closest_idx < 0) return false;

  bool corner = isCorner(integrate_path_vector, closest_idx);
  planVelocity(st, corner, stop_flag);
  GetLd(st);

  int target_idx = findWaypoint(integrate_path_vector, cav_x, cav_y, st.lookahead_m);
  if (target_idx < 0) target_idx = closest_idx; // fallback

  // Check bounds
  if (target_idx >= (int)integrate_path_vector.size()) target_idx = integrate_path_vector.size() - 1;

  double wz = PurePursuitAngularVelocity(target_idx, st.speed_mps, st.max_yaw_rate);

  // ===== 11. Publish Control (수정: 우선순위 재정렬) =====
  FillControlCommand(st, stop_flag, race_over, wz, node, cmd);
  return true;
}

// 1. 코너 주행 여부와 2. Stop Flag에 따른 속도를 결정한다.
void planVelocity(ControllerState& st, bool isCornerDetected, bool stop_flag) {

    // 1. 코너 주행 여부 확인
    if (isCornerDetected) {
        st.speed_mps = 1.3;
    } else {
        st.speed_mps = 2.0;
    }

    // 2. Stop Flag에 따른 속도 재조정 (Main 루프 하단에 있던 로직을 여기로 통합 추천)
    if (stop_flag) {
        if (current_lane == 2) {
            st.speed_mps = measured_hv24_vel; // Lane 2 막힘 -> 서행
        } else if (current_lane == 3) {
            st.speed_mps = measured_hv20_vel; // Lane 3 막힘 -> 약간 감속
        }
    }

    // 주의: ControllerState에 저장된 speed_mps는 나중에 wz(조향각) 계산과 cmd.linear.x에 사용됩니다.
}

// 현재 속도를 기반으로 Pure Pursuit의 Lookahead 거리를 계산한다.
void GetLd(ControllerState& st) {
    double gain_ld = 0.5;
    double max_ld = 0.35;
    double min_ld = 0.0;
    double ld = gain_ld * st.speed_mps;
    st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
}

// 목표 Waypoint와 현재 위치/자세로부터 곡률을 구하고, 이를 각속도로 변환한다.
double PurePursuitAngularVelocity(int target_idx, double speed_mps, double max_yaw_rate)
{
  const double dx = integrate_path_vector[target_idx].x - cav_x;
  const double dy = integrate_path_vector[target_idx].y - cav_y;

  const double cy = std::cos(cav_yaw);
  const double sy = std::sin(cav_yaw);
  const double x_v = cy * dx + sy * dy;
  const double y_v = -sy * dx + cy * dy;
  const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
  const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);

  double wz = speed_mps * kappa;
  wz = std::clamp(wz, -max_yaw_rate, max_yaw_rate);
  return wz;
}

// 우선순위(완주 > Zone1/2 충돌 > Stop Flag > 일반 주행) 순서로 최종 명령을 채운다.
void FillControlCommand(const ControllerState& st,
                         bool stop_flag, bool race_over,
                         double wz,
                         std::shared_ptr<rclcpp::Node> node,
                         geometry_msgs::msg::Accel& cmd)
{
  // [우선순위 1] Race Over (경기 종료 시 무조건 정지)
  if (race_over) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }
  // [우선순위 2] Zone 1 또는 Zone 2 충돌 감지 시 '무조건' 정지 (Stop Flag보다 먼저 검사해야 함!)
  else if (zone_collision_flag || zone2_collision_flag) {
    // 로그를 띄워서 정지 원인 확인 (과도한 로그 방지를 위해 Throttle 사용)
    RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 1000,
        "[EMERGENCY] Zone Collision! Stopping. (Zone1: %d, Zone2: %d)",
        zone_collision_flag, zone2_collision_flag);

    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }
  // [우선순위 3] 일반 Stop Flag (앞차 거리 유지 등) -> 감속하거나 정지
  else if (stop_flag) {
    // 각 차선별 감속 로직
    if (current_lane == 3 && zone2_collision_flag) {
        cmd.linear.x = 0.0; // 3차선은 완전 정지
        cmd.angular.z = 0.0;
    }
    else if (current_lane == 2) {
        cmd.linear.x = measured_hv24_vel; // HV24 속도 추종 (감속)
        cmd.angular.z = wz;
    }
    else if (current_lane == 3) {
        cmd.linear.x = measured_hv20_vel; // HV20 속도 추종 (감속)
        cmd.angular.z = wz;
    }
    else if (current_lane == 1) {
        cmd.linear.x = 0.0; // 1차선은 완전 정지
        cmd.angular.z = 0.0;
    }
    else {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
    }
  }
  // [우선순위 4] 그 외 일반 주행
  else {
    cmd.linear.x = st.speed_mps;
    cmd.angular.z = wz;
  }

  // unused fields
  cmd.linear.y = 0.0; cmd.linear.z = 0.0;
  cmd.angular.x = 0.0; cmd.angular.y = 0.0;
}

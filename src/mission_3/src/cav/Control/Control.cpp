/*
 * Control.hpp에 선언된 목표 속도 계획, 속도 램프, Pure Pursuit, 제어 명령 생성 함수들의 구현.
 */
#include "Control/Control.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "Planning/Planning.hpp"

// 목표 속도 결정, 속도 램프 적용, (Planning) 목표 Waypoint 재탐색, Pure Pursuit 각속도 계산,
// 최종 명령 생성을 순서대로 수행한다.
void ControlProcess(ControllerState& st,
                     bool corner_detected,
                     const std::vector<integrate_path_struct>& path,
                     double x_m, double y_m, double yaw,
                     rclcpp::Time stamp,
                     bool mission_completed,
                     int actual_cav_id,
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

  // 속도 램프 적용 - 급가속 방지 및 헤딩 안정화
  double ramped_speed = applyVelocityRamp(st, current_target_speed, stamp);

  st.speed_mps = current_target_speed;  // 목표값 유지

  // 목표 속도가 반영된 Lookahead로 목표 Waypoint를 다시 탐색한다.
  int target_path_idx = FindTargetWaypoint(path, x_m, y_m, st);

  // ramped_speed를 사용하여 각속도도 부드럽게 제어한다.
  double wz = PurePursuitAngularVelocity(path, target_path_idx, x_m, y_m, yaw,
                                          ramped_speed, st.max_yaw_rate);

  FillControlCommand(st, ramped_speed, wz, stamp, mission_completed, actual_cav_id, cmd, twist_cmd);
}

// 직선(1.8)/커브(1.5) 구간에 따라 목표 속도를 정한다.
// 실측 랩타임 기준 1.7 m/s -> 1:05, 1.8 m/s -> 1:04, 2.0 m/s -> 1:00으로 1.8 m/s를 채택했다.
void planVelocity(ControllerState& st, bool isCorner) {
    if (!isCorner) {st.speed_mps = 1.8; } else { st.speed_mps = 1.5; }
}

// 최대 가속도 제한으로 목표 속도까지 부드럽게 도달시킨다. Red flag 해제 직후에는
// 더 느린 가속도를 적용하여 헤딩이 안정화될 시간을 준다.
double applyVelocityRamp(ControllerState& st, double target_velocity, rclcpp::Time current_time) {
    // 첫 호출 시 초기화
    if (st.last_update_time.nanoseconds() == 0) {
        st.last_update_time = current_time;
        st.current_velocity = 0.0;
        return 0.0;
    }

    // 경과 시간 계산
    double dt = (current_time - st.last_update_time).seconds();
    if (dt < 0.001) dt = 0.001;  // 최소 1ms
    if (dt > 0.1) dt = 0.2;      // 최대 100ms (센서 끊김 방지)

    st.last_update_time = current_time;

    // 목표 속도와 현재 속도의 차이
    double velocity_diff = target_velocity - st.current_velocity;

    // Red flag 해제 후 일시적으로 낮은 가속도 사용
    double effective_max_accel = st.max_acceleration;

    if (st.red_flag == 0 && st.red_flag_hold_count < st.RED_FLAG_RELEASE_STABILIZE_CYCLES) {
        // Red flag가 해제되었지만, 안정화 기간 중
        st.red_flag_hold_count++;
        effective_max_accel = st.max_acceleration_after_red_flag;  // 느린 가속도 적용
    } else if (st.red_flag == 1) {
        // Red flag 중: 카운트 리셋
        st.red_flag_hold_count = 0;
    }

    // 최대 가속도에 따른 속도 변화량 계산
    double max_delta_v = effective_max_accel * dt;

    // 속도 제한 (급격한 변화 방지)
    if (velocity_diff > max_delta_v) {
        st.current_velocity += max_delta_v;
    } else if (velocity_diff < -max_delta_v) {
        st.current_velocity -= max_delta_v;
    } else {
        st.current_velocity = target_velocity;
    }

    // 속도가 음수가 되지 않도록 제한
    st.current_velocity = std::max(0.0, st.current_velocity);

    return st.current_velocity;
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

  /*
  kappa - 곡률
  ws(각속도) - 속도 * 곡률 = 회전각도
  clamp를 통해, 최대 조향을 제한
  마지막으로 twist.cmd.angular_z = ws 로 각속도를 전달
  */
}

// 시작 3초 정지 -> 완주 -> Red Flag -> Yellow Flag -> 일반 주행 순서의 우선순위로 명령을 채운다.
void FillControlCommand(ControllerState& st,
                         double ramped_speed, double wz,
                         rclcpp::Time stamp,
                         bool mission_completed,
                         int actual_cav_id,
                         geometry_msgs::msg::Accel& cmd,
                         geometry_msgs::msg::Twist& twist_cmd)
{
  // 시작 3초 정지 체크
  if (st.is_start_stop_active) {
      double elapsed_ms = (stamp - st.start_stop_time).seconds() * 1000.0;
      if (elapsed_ms < 3000) {
          // 3초 동안 정지
          cmd.linear.x = 0.0;
          cmd.angular.z = 0.0;
          twist_cmd.linear.x = 0.0;
          twist_cmd.angular.z = 0.0;
      } else {
          // 3초 경과 후 정지 해제
          st.is_start_stop_active = false;
          std::cout << "[CAV_index " << actual_cav_id << "] 3-second initial stop completed, starting motion" << std::endl;
      }
  }

  // 정지 중이 아니면 정상 제어 진행
  if (!st.is_start_stop_active) {
      // Check if ALL CAVs have finished 5 laps
      if (mission_completed) {
      // All CAVs finished 5 laps - STOP
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      twist_cmd.linear.x = 0.0; // Stop command ->  바퀴 힘 없이 , 관성으로 랩타임 넘어가게끔,
      twist_cmd.angular.z = 0.0;
  } else if (st.red_flag == 1) {
      // Red flag: STOP with -0.005
      cmd.linear.x  = 0.0;
      cmd.angular.z = 0.0;
      twist_cmd.linear.x = -0.005; // Stop command
      twist_cmd.angular.z = 0.0;
  } else if (st.yellow_flag == 1) {
      // Yellow flag Group 1 (Zone 1,2): Slow down to 0.8 m/s
      cmd.linear.x  = 0.8;
      twist_cmd.linear.x = 0.8;
      cmd.angular.z = wz;
      twist_cmd.angular.z = wz;
  } else if (st.yellow_flag == 2) {
      // Yellow flag Group 2 (Zone 3,4,5,6,7,8): Slow down to 1.5 m/s
      cmd.linear.x  = 0.8;
      twist_cmd.linear.x = 0.8;
      cmd.angular.z = wz;
      twist_cmd.angular.z = wz;
  } else {
      cmd.linear.x  = ramped_speed;
      cmd.angular.z = wz;
      twist_cmd.linear.x = ramped_speed;
      twist_cmd.angular.z = wz;
  }
      } // 정지 중이 아닐 때 제어 끝

  cmd.linear.y = 0.0; cmd.linear.z = 0.0;
  cmd.angular.x = 0.0; cmd.angular.y = 0.0;

  twist_cmd.linear.y = 0.0; twist_cmd.linear.z = 0.0;
  twist_cmd.angular.x = 0.0; twist_cmd.angular.y = 0.0;
}

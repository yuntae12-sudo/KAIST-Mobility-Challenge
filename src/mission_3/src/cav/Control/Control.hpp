/*
 * Mission 3 차량의 목표 속도 계획, 속도 램프, Pure Pursuit 각속도 계산,
 * 시작 정지/Red·Yellow Flag/완주 상태를 반영한 제어 명령 생성을 담당한다.
 */
#ifndef MISSION_3_CAV_CONTROL_HPP
#define MISSION_3_CAV_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Global/Global.hpp"

// ==============================
// Control Process
// ==============================
// 목표 속도 결정 -> 속도 램프 적용 -> (Planning) 목표 Waypoint 반영 Lookahead 갱신 -> Pure Pursuit ->
// 우선순위(시작 정지 > 완주 > Red Flag > Yellow Flag > 일반 주행) 기반 명령 생성을 수행한다.
void ControlProcess(ControllerState& st,
                     bool corner_detected,
                     const std::vector<integrate_path_struct>& path,
                     double x_m, double y_m, double yaw,
                     rclcpp::Time stamp,
                     bool mission_completed,
                     int actual_cav_id,
                     geometry_msgs::msg::Accel& cmd,
                     geometry_msgs::msg::Twist& twist_cmd);

// ==============================
// Velocity Planning
// ==============================
void planVelocity(ControllerState& st, bool isCorner);
double applyVelocityRamp(ControllerState& st, double target_velocity, rclcpp::Time current_time);

// ==============================
// Pure Pursuit
// ==============================
double PurePursuitAngularVelocity(const std::vector<integrate_path_struct>& path,
                                   int target_path_idx,
                                   double x_m, double y_m, double yaw,
                                   double speed, double max_yaw_rate);

// ==============================
// Command Publish
// ==============================
void FillControlCommand(ControllerState& st,
                         double ramped_speed, double wz,
                         rclcpp::Time stamp,
                         bool mission_completed,
                         int actual_cav_id,
                         geometry_msgs::msg::Accel& cmd,
                         geometry_msgs::msg::Twist& twist_cmd);

#endif  // MISSION_3_CAV_CONTROL_HPP

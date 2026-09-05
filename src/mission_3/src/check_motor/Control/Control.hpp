/*
 * check_motor 차량의 목표 속도 계획, Pure Pursuit 각속도 계산,
 * 완주/Red Flag/Yellow Flag 상태를 반영한 제어 명령 생성을 담당한다.
 */
#ifndef MISSION_3_CHECK_MOTOR_CONTROL_HPP
#define MISSION_3_CHECK_MOTOR_CONTROL_HPP

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "Global/Global.hpp"

// ==============================
// Control Process
// ==============================
// 목표 속도 결정 -> (Planning) 목표 Waypoint 반영 Lookahead 갱신 -> Pure Pursuit ->
// 우선순위(완주 > Red Flag > Yellow Flag > 일반 주행) 기반 명령 생성을 수행한다.
void ControlProcess(ControllerState& st,
                     bool corner_detected,
                     const std::vector<integrate_path_struct>& path,
                     double x_m, double y_m, double yaw,
                     bool mission_completed,
                     geometry_msgs::msg::Accel& cmd,
                     geometry_msgs::msg::Twist& twist_cmd);

// ==============================
// Velocity Planning
// ==============================
void planVelocity(ControllerState& st, bool isCorner);

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
void FillControlCommand(const ControllerState& st,
                         double target_speed, double wz,
                         bool mission_completed,
                         geometry_msgs::msg::Accel& cmd,
                         geometry_msgs::msg::Twist& twist_cmd);

#endif  // MISSION_3_CHECK_MOTOR_CONTROL_HPP

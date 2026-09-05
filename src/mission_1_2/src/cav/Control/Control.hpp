/*
 * Mission 1-2 CAV의 목표 속도 계획, Pure Pursuit 각속도 계산,
 * Red Flag 반영 및 제어 명령 생성을 담당한다.
 */
#ifndef MISSION_1_2_CONTROL_HPP
#define MISSION_1_2_CONTROL_HPP

#include <geometry_msgs/msg/accel.hpp>

#include "Global/Global.hpp"

// ==============================
// Control Process
// ==============================
// 커브 감지 결과와 타워 모드 여부로 목표 속도를 정하고, 그 값을 st.speed_mps에 반영한다.
double DecideTargetSpeed(ControllerState& st, bool corner_detected);

// 목표 Waypoint를 향한 각속도를 계산하고, Red Flag를 반영한 제어 명령을 채운다.
void ControlProcess(const ControllerState& st,
                     double target_speed,
                     const std::vector<integrate_path_struct>& path,
                     int target_path_idx,
                     double x_m, double y_m, double yaw,
                     geometry_msgs::msg::Accel& cmd);

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
                                   double target_speed, double max_yaw_rate);

// ==============================
// Command Publish
// ==============================
void FillControlCommand(const ControllerState& st,
                         double target_speed, double wz,
                         geometry_msgs::msg::Accel& cmd);

#endif  // MISSION_1_2_CONTROL_HPP

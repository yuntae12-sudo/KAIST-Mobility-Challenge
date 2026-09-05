/*
 * Mission 2 차량의 목표 속도 계획, Pure Pursuit 각속도 계산, 우선순위 기반
 * 제어 명령 생성을 담당한다.
 */
#ifndef MISSION_2_CONTROL_HPP
#define MISSION_2_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/accel.hpp>

#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Control Process
// ==============================
// Closest Point/커브 판단 -> 목표 속도 계획 -> Pure Pursuit -> 우선순위 기반 명령 생성을 순서대로 수행한다.
// Closest Point를 찾지 못하면 cmd를 채우지 않고 false를 반환한다 (호출자는 publish를 생략해야 한다).
bool ControlProcess(ControllerState& st,
                     bool stop_flag, bool race_over,
                     std::shared_ptr<rclcpp::Node> node,
                     geometry_msgs::msg::Accel& cmd);

// ==============================
// Velocity Planning
// ==============================
void planVelocity(ControllerState& st, bool isCornerDetected, bool stop_flag);
void GetLd(ControllerState& st);

// ==============================
// Pure Pursuit
// ==============================
double PurePursuitAngularVelocity(int target_idx, double speed_mps, double max_yaw_rate);

// ==============================
// Command Publish
// ==============================
void FillControlCommand(const ControllerState& st,
                         bool stop_flag, bool race_over,
                         double wz,
                         std::shared_ptr<rclcpp::Node> node,
                         geometry_msgs::msg::Accel& cmd);

#endif  // MISSION_2_CONTROL_HPP

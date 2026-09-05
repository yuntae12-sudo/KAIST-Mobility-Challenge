/*
 * CAV ROI와 HV ROI를 짝지어 HV 도착 여부에 따라 CAV의 RED_FLAG/target_vel을 발행하고,
 * HV 순간 속도 측정을 담당한다.
 */
#ifndef MISSION_3_ROTARY_ROTARY_HPP
#define MISSION_3_ROTARY_ROTARY_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

#include <map>
#include <memory>

#include "Global/Global.hpp"

// ==============================
// Rotary Process (ROI Pair 기반 통합 제어)
// ==============================
// cav_roi_id에 CAV가 접근하면, 짝지어진 hv_roi_id에 HV가 도착했는지 확인하여
// 도착했으면 통과 허가(GO)를, 아니면 정지(STOP)를 발행한다. 허가증은 CAV가
// reset_radius를 벗어날 때까지 유지된다.
void monitor_all_rois(
    int cav_roi_id, int hv_roi_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>& target_vel_pubs,
    double detection_radius,
    double reset_radius
);

// ==============================
// HV Velocity Measurement
// ==============================
void CalculateInstantVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                              int hv_id,
                              std::map<int, HVState>& states);

#endif  // MISSION_3_ROTARY_ROTARY_HPP

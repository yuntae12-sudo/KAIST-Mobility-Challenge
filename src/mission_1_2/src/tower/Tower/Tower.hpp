/*
 * 두 CAV의 위치를 바탕으로 Zone/ROI 충돌 위험을 판단하고,
 * RED_FLAG 신호를 발행하는 Control Tower의 핵심 판단 로직을 담당한다.
 */
#ifndef MISSION_1_2_TOWER_TOWER_HPP
#define MISSION_1_2_TOWER_TOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <map>
#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Tower Process
// ==============================
// ROI 기반 CAV1/CAV2 제어 판단 -> Zone 2/4 모니터링 순서로 수행하는 상위 Process 함수.
// main의 while 루프가 매 tick마다 호출해야 하는 Tower 판단 전체를 대표한다.
void TowerProcess(
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance);

// ==============================
// Zone Check
// ==============================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius);
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius);

// ==============================
// ROI 기반 CAV 제어 판단
// ==============================
bool should_cav2_stop_by_roi(std::shared_ptr<rclcpp::Node> node);
bool should_cav1_stop_by_merge_roi(std::shared_ptr<rclcpp::Node> node);

// ==============================
// Zone 모니터링 및 RED_FLAG 발행
// ==============================
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance);

#endif  // MISSION_1_2_TOWER_TOWER_HPP

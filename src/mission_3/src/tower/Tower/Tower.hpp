/*
 * 최대 4대 CAV의 위치를 바탕으로 3개 Zone의 충돌 위험을 판단하고,
 * RED_FLAG 신호를 발행하는 Control Tower의 핵심 판단 로직을 담당한다.
 */
#ifndef MISSION_3_TOWER_TOWER_HPP
#define MISSION_3_TOWER_TOWER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

#include <map>
#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Zone Check
// ==============================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius);
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius);

// ==============================
// Zone 모니터링 및 RED_FLAG 발행 (Tower Process)
// ==============================
// CAV 인덱스(1~4)와 실제 CAV ID(CAV_IDS 환경변수 순서) 사이의 매핑을 사용해
// 경로/Publisher를 조회하고 RED_FLAG 신호를 발행한다.
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance,
    const std::map<int, int>& cav_index_map_actual_to_idx,
    const std::map<int, int>& cav_index_map_idx_to_actual);

#endif  // MISSION_3_TOWER_TOWER_HPP

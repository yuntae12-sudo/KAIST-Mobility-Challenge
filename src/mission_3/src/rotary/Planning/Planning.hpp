/*
 * CAV 위치를 바탕으로 경로 인덱스를 탐색하고, Yellow ROI 기반 Zone Group을 판단한다.
 */
#ifndef MISSION_3_ROTARY_PLANNING_HPP
#define MISSION_3_ROTARY_PLANNING_HPP

#include <vector>

#include "Global/Global.hpp"

// ==============================
// Path Search
// ==============================
int find_closest_waypoint_index(const std::vector<Pose>& path, Pose current_pose);

// ==============================
// Zone Group Judgement
// ==============================
// CAV가 현재 어떤 Yellow ROI Zone Group에 속해 있는지 판단한다 (0 = 어디에도 속하지 않음).
int get_zone_group_for_cav(int cav_index, const Pose& cav_pose, double detection_radius);

#endif  // MISSION_3_ROTARY_PLANNING_HPP

/*
 * 각 CAV의 저장된 경로(cav_paths)를 바탕으로 Lookahead 구간을 추출하고,
 * 두 CAV 간 경로 겹침 여부와 Sub CAV 선정을 담당한다.
 */
#ifndef MISSION_3_TOWER_PLANNING_HPP
#define MISSION_3_TOWER_PLANNING_HPP

#include <vector>

#include "Global/Global.hpp"

// ==============================
// Waypoint Search
// ==============================
int find_closest_waypoint_index(const std::vector<Pose>& path, Pose current_pose);
std::vector<Pose> get_lookahead_waypoints(const std::vector<Pose>& path, int current_idx, int lookahead_count = 200);

// ==============================
// Path Overlap
// ==============================
bool check_path_overlap(const std::vector<Pose>& main_lookahead,
                         const std::vector<Pose>& sub_lookahead,
                         double overlap_threshold,
                         double& csv_distance);
bool is_moving_away(int cav_id, Pose zone_origin, const std::vector<Pose>& path);

// ==============================
// Sub CAV Selection
// ==============================
int select_best_sub_cav(int main_cav_id, const std::vector<int>& precollision_candidates,
                         Pose zone_origin, double overlap_threshold, int lookahead_distance);

#endif  // MISSION_3_TOWER_PLANNING_HPP

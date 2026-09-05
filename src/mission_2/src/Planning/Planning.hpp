/*
 * Mission 2 차량의 차선 선택, 경로 인덱스 탐색, 겹침 구간 판정을 담당한다.
 */
#ifndef MISSION_2_PLANNING_HPP
#define MISSION_2_PLANNING_HPP

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Planning Process
// ==============================
// 각 차선의 Closest Index 갱신, Overlap 강제 차선 조정, 차선 선택/전환을 순서대로 수행한다.
// stop_flag는 Mission에서 계산된 정지 필요 여부이며, 차선 전환 허용 조건에 사용된다.
int PlanningProcess(int cav_id,
                     const std::vector<bool>& lane_collision,
                     bool zone3_flag, bool zone4_flag, bool zone5_flag,
                     bool zone_collision_flag_in, bool zone2_collision_flag_in,
                     bool stop_flag,
                     std::shared_ptr<rclcpp::Node> node);

// ==============================
// Path Search
// ==============================
int get_lane_start_idx(int lane_id, double cav_x, double cav_y);
int findClosestPointSimple(const std::vector<PathPoint>& path, double x, double y);
int findClosestPointAhead(const std::vector<PathPoint>& path, double x, double y, int ref_idx);
int findClosestPoint(const std::vector<PathPoint>& path, double x, double y, int lane);
int findWaypoint(const std::vector<PathPoint>& path, double x, double y, double lookahead);
bool isCorner(const std::vector<PathPoint>& path, int closest_idx);

// ==============================
// Overlap Region
// ==============================
void init_overlap_region();
bool check_approaching_overlap(int current_lane3_idx);

// ==============================
// Lane Selection
// ==============================
int choose_lane(const std::vector<bool>& collision_list, int current_lane,
                 bool in_zone3, bool in_zone4, bool in_zone5);
void change_csv_state(int cav_id, int new_lane, std::shared_ptr<rclcpp::Node> node);

#endif  // MISSION_2_PLANNING_HPP

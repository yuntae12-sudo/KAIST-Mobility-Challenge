/*
 * check_motor 차량의 경로 추종을 위한 목표 Waypoint 탐색과 Lookahead 계산을 담당한다.
 * 목표 속도가 정해진 이후(Control)에 Lookahead가 정해지므로,
 * Closest Point/Corner 판단과 목표 Waypoint 탐색을 두 단계로 나누어 제공한다.
 */
#ifndef MISSION_3_CHECK_MOTOR_PLANNING_HPP
#define MISSION_3_CHECK_MOTOR_PLANNING_HPP

#include <vector>

#include "Global/Global.hpp"

// ==============================
// Planning Process
// ==============================
// 현재 위치에서 Closest Point를 찾고, 그 지점 기준으로 커브 여부를 판단한다.
void PlanningProcess(const std::vector<integrate_path_struct>& path,
                      double x_m, double y_m,
                      const ControllerState& st,
                      int& closest_idx,
                      bool& corner_detected);

// 목표 속도가 반영된 Lookahead 거리로 목표 Waypoint 인덱스를 계산한다.
int FindTargetWaypoint(const std::vector<integrate_path_struct>& path,
                        double x_m, double y_m,
                        ControllerState& st);

// ==============================
// Path Search
// ==============================
int findClosestPoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m);
int findWaypoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m, double L_d);
bool isCorner(const std::vector<integrate_path_struct>& path, double L_d, int closest_idx);

// ==============================
// Lookahead
// ==============================
void GetLd(ControllerState& st);

#endif  // MISSION_3_CHECK_MOTOR_PLANNING_HPP

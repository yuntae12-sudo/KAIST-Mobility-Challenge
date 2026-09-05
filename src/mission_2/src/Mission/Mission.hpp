/*
 * Mission 2의 Zone/ROI 충돌 판단, 차선별 정지 필요 여부, HV 속도 측정,
 * 완주(Race Over) 판정을 담당한다.
 */
#ifndef MISSION_2_MISSION_HPP
#define MISSION_2_MISSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Mission Process
// ==============================
// Zone 1~5 충돌 플래그를 갱신하고, 차선별 물리적 충돌 여부를 계산한 뒤 정지 필요 여부를 판단한다.
// lane_collision(출력)은 인덱스 1~3이 각 차선의 최종 충돌 상태를 나타낸다.
bool MissionProcess(int cav_id, std::vector<bool>& lane_collision, bool& is_overlap_zone,
                     std::shared_ptr<rclcpp::Node> node);

// ==============================
// Zone 1 Detection
// ==============================
bool is_hv_in_zone(double hv_x, double hv_y);
bool is_cav_in_zone(double cav_x_in, double cav_y_in);
bool check_zone_collision();

// ==============================
// Zone 2 Detection
// ==============================
bool is_hv_in_zone2(double hv_x, double hv_y);
bool is_cav_in_zone2(double cav_x_in, double cav_y_in);
bool check_zone2_collision();

// ==============================
// Zone 3 & 4 & 5 Detection
// ==============================
bool is_cav_in_zone3(double cav_x_in, double cav_y_in);
bool is_cav_in_zone3_1(double cav_x_in, double cav_y_in);
bool is_cav_in_zone3_2(double cav_x_in, double cav_y_in);
bool is_cav_in_zone4(double cav_x_in, double cav_y_in);
bool is_cav_in_zone5(double cav_x_in, double cav_y_in);

// ==============================
// Lane Collision / Stop Decision
// ==============================
bool is_collision(int idx, const std::vector<PathPoint>& path, double threshold = 0.15);
bool check_lane_roi_collision(int lane_id, int start_idx, const std::vector<PathPoint>& path);
bool should_stop(const std::vector<bool>& collision_list, int current_lane);

// ==============================
// HV Velocity Measurement
// ==============================
void MeasureHVVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                       int hv_id,
                       std::shared_ptr<rclcpp::Node> node);

// ==============================
// Race Over
// ==============================
bool is_race_over(CavState& cav);

#endif  // MISSION_2_MISSION_HPP

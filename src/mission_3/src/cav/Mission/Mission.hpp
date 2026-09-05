/*
 * Mission 3 CAV들의 Lap 카운트, 5바퀴 완주 판정, 전체 완주 여부,
 * 시작 3초 정지 활성화를 담당한다.
 */
#ifndef MISSION_3_CAV_MISSION_HPP
#define MISSION_3_CAV_MISSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <memory>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Mission Process
// ==============================
// 시작점 복귀 여부로 해당 CAV의 Lap을 갱신하고, 최초 초기화 시 시작 3초 정지를 활성화한다.
void MissionProcess(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id,
                     std::vector<CavState>& states, int my_cav_index,
                     std::shared_ptr<ControllerState> st);

// ==============================
// Mission Completion
// ==============================
bool CheckAllFinished(const std::vector<CavState>& cav_list, int vehicle_count);

#endif  // MISSION_3_CAV_MISSION_HPP

/*
 * Mission 1-2 CAV들의 Lap 카운트, 5바퀴 완주 판정, 전체 완주 여부 판단을 담당한다.
 */
#ifndef MISSION_1_2_MISSION_HPP
#define MISSION_1_2_MISSION_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>

#include "Global/Global.hpp"

// ==============================
// Mission Process
// ==============================
// 시작점 복귀 여부로 해당 CAV의 Lap을 갱신하고, 5바퀴 완주 시 finished를 설정한다.
void MissionProcess(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id, std::vector<CavState>& states);

// ==============================
// Mission Completion
// ==============================
// 자신을 제외한 모든 CAV가 완주했는지 확인한다.
bool CheckAllFinished(const std::vector<CavState>& cav_list);

#endif  // MISSION_1_2_MISSION_HPP

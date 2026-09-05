/*
 * Mission 1-1 차량의 Lap 카운트 및 5바퀴 완주 판정을 담당한다.
 */
#ifndef MISSION_1_1_MISSION_HPP
#define MISSION_1_1_MISSION_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "Global/Global.hpp"

// ==============================
// Mission Process
// ==============================
// 시작점 복귀 여부로 Lap을 갱신하고, 5바퀴 완주 시 finished를 설정한다.
void MissionProcess(const geometry_msgs::msg::PoseStamped::SharedPtr msg, CavState& current_cav);

#endif  // MISSION_1_1_MISSION_HPP

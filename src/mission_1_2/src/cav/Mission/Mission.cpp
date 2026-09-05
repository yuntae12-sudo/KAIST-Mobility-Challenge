/*
 * Mission.hpp에 선언된 Lap/완주 판단 함수들의 구현.
 */
#include "Mission/Mission.hpp"

#include <cmath>
#include <iostream>

// 시작점으로부터 0.1m 이내 재진입을 Lap 완료로 판단하고, 5랩 도달 시 완주 처리한다.
void MissionProcess(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id, std::vector<CavState>& states) {
    CavState& current_cav = states[cav_id];
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    if (!current_cav.is_initialized) {
        current_cav.start_x = current_x;
        current_cav.start_y = current_y;
        current_cav.is_initialized = true;
        current_cav.is_in_zone = true;
        std::cout << "[CAV " << cav_id << "] Start Point Set: (" << current_x << ", " << current_y << ")" << std::endl << std::endl;
        return;
    }

    if (current_cav.finished) return;

    double dist_to_start = std::hypot(current_x - current_cav.start_x, current_y - current_cav.start_y);

    if (dist_to_start < 0.1) {
        if (!current_cav.is_in_zone) {
            current_cav.current_lap += 1;
            current_cav.is_in_zone = true;
            std::cout << "[CAV " << cav_id << "] Lap Increased! Current Lap: " << current_cav.current_lap << std::endl << std::endl;
            if (current_cav.current_lap >= 5) {
                current_cav.finished = true;
                std::cout << "[CAV " << cav_id << "] FINISHED 5 LAPS!" << std::endl << std::endl;
            }
        }
    } else {
        if (current_cav.is_in_zone) current_cav.is_in_zone = false;
    }
}

// 자신을 제외한 모든 CAV의 완주 여부를 확인한다 (cav_list.size()는 total_vehicle_count + 1).
bool CheckAllFinished(const std::vector<CavState>& cav_list) {
    int finished_count = 0;
    for (int i = 0; i < (int)cav_list.size(); ++i) {
        if (cav_list[i].finished) finished_count++;
    }
    if (finished_count == (int)cav_list.size() - 1) return true;
    return false;
}

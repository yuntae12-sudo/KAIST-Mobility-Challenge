/*
 * Mission.hpp에 선언된 Lap/완주 판단 및 시작 정지 활성화 함수들의 구현.
 */
#include "Mission/Mission.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

// 자신을 제외한 모든 CAV의 완주 여부를 확인한다.
// vehicle_count 파라미터는 4대 고정 운영 체제로 변경되며 더 이상 사용되지 않는다 (원본 그대로 보존).
bool CheckAllFinished(const std::vector<CavState>& cav_list, int /*vehicle_count*/) {
    int finished_count = 0;
    for (int i = 1; i <= 4; ++i) {
        if (cav_list[i].finished) finished_count++;
    }
    if (finished_count == 4) return true;
    return false;
}

// 시작점으로부터 0.3m 이내 재진입을 Lap 완료로 판단하고, 6랩(=5바퀴 완주) 도달 시 완주 처리한다.
// 최초 초기화 시 시작 3초 정지를 함께 활성화한다.
void MissionProcess(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id,
                     std::vector<CavState>& states, int my_cav_index,
                     std::shared_ptr<ControllerState> st) {
    CavState& current_cav = states[cav_id];
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    bool is_my_cav = (cav_id == my_cav_index);  // 자신의 CAV인지 확인

    if (!current_cav.is_initialized) {
        current_cav.start_x = current_x;
        current_cav.start_y = current_y;
        current_cav.is_initialized = true;
        current_cav.is_in_zone = false;  // ← FALSE로 시작! 첫 프레임에서 영역 밖으로 설정
        current_cav.lap_start_time = msg->header.stamp;

        // 시작 3초 정지 활성화
        st->is_start_stop_active = true;
        st->start_stop_time = msg->header.stamp;
        if (is_my_cav) {
            std::cout << "[CAV_index " << cav_id << "] 3-second initial stop activated" << std::endl;
        }

        return;
    }

    if (current_cav.finished) return;

    double dist_to_start = std::hypot(current_x - current_cav.start_x, current_y - current_cav.start_y);

    if (dist_to_start < 0.3) { // 0.15 -> 0.3
        if (!current_cav.is_in_zone) {
            current_cav.current_lap += 1;
            current_cav.is_in_zone = true;

            // Calculate lap time
            double lap_time_sec = (rclcpp::Time(msg->header.stamp) - current_cav.lap_start_time).seconds();
            current_cav.lap_start_time = rclcpp::Time(msg->header.stamp);  // Reset for next lap

            if (is_my_cav) {
                std::cout << "═══════════════════════════════════════════════════════════════" << std::endl;
                std::cout << "🏁 [CAV_index " << cav_id << "] Lap: " << current_cav.current_lap << "/5 (";
                std::cout << (current_cav.current_lap * 100 / 5) << "% Complete)" << std::endl;
                std::cout << "   Lap Time: " << std::fixed << std::setprecision(2) << lap_time_sec << " sec" << std::endl;
                std::cout << "   Current Pos: (" << std::fixed << std::setprecision(2) << current_x << ", " << current_y << ")" << std::endl;
                std::cout << "═══════════════════════════════════════════════════════════════" << std::endl << std::endl;
            }

            if (current_cav.current_lap >= 6) { // ** LAP COUNT 5 -> 6 **
                current_cav.finished = true;
                if (is_my_cav) {
                    std::cout << "🎉 [CAV_index " << cav_id << "] ✓ FINISHED 5 LAPS! ✓" << std::endl << std::endl;
                }
            }
        }
    } else {
        if (current_cav.is_in_zone) {
            current_cav.is_in_zone = false;
        }
    }
}

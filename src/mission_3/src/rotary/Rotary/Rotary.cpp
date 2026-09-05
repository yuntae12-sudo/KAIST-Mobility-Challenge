/*
 * Rotary.hpp에 선언된 ROI Pair 기반 CAV 통행 허가 판단과 HV 속도 측정 함수들의 구현.
 */
#include "Rotary/Rotary.hpp"

#include <cmath>
#include <numeric>

#include "Utils/Utils.hpp"

// =========================
// 통합 제어 함수 Pair 기반
// =========================
void monitor_all_rois(
    int cav_roi_id, int hv_roi_id,
    std::shared_ptr<rclcpp::Node> /*node*/,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>& target_vel_pubs,
    double detection_radius,  // cav_roi에서는 저 범위 안에 들어오면 cav 멈추게, hv_roi에서는 hv가 통과할 때 cav에게 주행권한 주는 범위
    double reset_radius
) {
    Pose cav_origin = cav_rois[cav_roi_id];
    Pose hv_origin = hv_rois[hv_roi_id];

    // 현재 검사 중인 ROI의 허가증 목록만 가져옴
    std::set<int>& permit_list = roi_permit_vehicles[cav_roi_id];

    for (const auto& [cav_id, pose] : cav_poses) {

        double dist = calculate_distance(pose, cav_origin);

        auto msg = std_msgs::msg::Int32();
        auto vel_msg = std_msgs::msg::Float64();
        bool send_commend = false;
        bool has_permission = permit_list.count(cav_id);

        // 1. 리셋 로직 (버퍼 벗어나면)
        if (dist > reset_radius) {
            if (has_permission) {
                permit_list.erase(cav_id); // 내 구역 허가증만 회수

                vel_msg.data = -1.0; // 속도 명령 리셋 신호

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);

            }
            continue;
        }


        // (Case A) 내 구역 허가증이 있는지 체크
        if (has_permission) {
            msg.data = 0; // GO

            send_commend = true;
            if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
            if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
        }

        // (Case B) 허가증 없으면 감지 및 검사
        else if (dist <= detection_radius) {
            bool hv_here = false;
            for (const auto& [hv_id, hv_pose] : hv_poses) {
                if (calculate_distance(hv_pose, hv_origin) <= detection_radius) {
                    hv_here = true;
                    break;
                }
            }

            if (hv_here) {
                // HV 도착 -> 내 구역에 허가증 발급
                permit_list.insert(cav_id);
                msg.data = 0;
                send_commend = true;

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
            }
            else {
                msg.data = 1; // STOP
                vel_msg.data = 0.0;

                if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);
                if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
            }
        }
        // 2. 명령 발행
        if (send_commend) {
            vel_msg.data = measured_hv_vel;

            if(red_flag_pubs.count(cav_id)) red_flag_pubs[cav_id]->publish(msg);
            if(target_vel_pubs.count(cav_id)) target_vel_pubs[cav_id]->publish(vel_msg);

        }
    }
}

// ==========================================
// 즉각적인 속도 측정 함수
// ==========================================
void CalculateInstantVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                              int hv_id,
                              std::map<int, HVState>& states)
{
    HVState& state = states[hv_id];
    rclcpp::Time current_time = msg->header.stamp;
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    // 1. 첫 메시지면 초기화만 하고 리턴
    if (state.is_first_msg) {
        state.prev_x = current_x;
        state.prev_y = current_y;
        state.last_time = current_time;
        state.is_first_msg = false;
        return;
    }

    // 2. 시간 변화량 (dt) 계산
    double dt = (current_time - state.last_time).seconds();

    // dt가 너무 작으면(0.001초 미만) 계산 스킵 (Division by zero 방지 및 노이즈 감소)
    if (dt < 0.001) return;

    // 3. 이동 거리 (dist) 계산
    double dx = current_x - state.prev_x;
    double dy = current_y - state.prev_y;
    double dist = std::hypot(dx, dy);

    // 4. 순간 속도 계산 (v = d / t)
    double current_vel = dist / dt;

    // 5. 이동 평균 필터 (Moving Average)
    // 버퍼에 현재 속도 추가
    if (state.vel_buffer.size() >= state.buffer_size) {
        state.vel_buffer.pop_front();
    }
    state.vel_buffer.push_back(current_vel);

    // 버퍼 평균 계산
    double sum = std::accumulate(state.vel_buffer.begin(), state.vel_buffer.end(), 0.0);
    double avg_vel = sum / state.vel_buffer.size();

    // 6. 전역 변수 업데이트 (혹은 필요한 곳에 사용)
    // 노이즈로 인한 미세한 움직임은 0으로 처리 (Optional)
    if (avg_vel < 0.01) avg_vel = 0.0;

    // 최대 속도 제한 (시뮬레이터 튀는 현상 방지)
    if (avg_vel > 2.5) avg_vel = 2.5;

    measured_hv_vel = avg_vel;

    // 7. 상태 업데이트
    state.prev_x = current_x;
    state.prev_y = current_y;
    state.last_time = current_time;
}

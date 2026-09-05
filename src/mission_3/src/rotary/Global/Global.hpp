/*
 * Mission 3 Control Rotary 노드 전체에서 공유되는 자료구조와 ROI/Zone Group 전역 상태를 정의한다.
 * Planning/Rotary/Visualizer 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_3_ROTARY_GLOBAL_HPP
#define MISSION_3_ROTARY_GLOBAL_HPP

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <map>
#include <set>
#include <vector>

// =========================
// 구조체 정의
// =========================
struct Pose {
  double x;
  double y;
};

struct HVState {
    int id;

    // 즉각적인 속도 측정을 위해 필요한 변수들
    bool is_first_msg = true;     // 첫 메시지인지 확인용
    double prev_x = 0.0;          // 이전 X 좌표
    double prev_y = 0.0;          // 이전 Y 좌표
    rclcpp::Time last_time;       // 이전 메시지 수신 시간

    // 속도 노이즈 제거를 위한 버퍼 (최근 10개 데이터 평균)
    std::deque<double> vel_buffer;
    const size_t buffer_size = 10;

    // 기존 변수들 (혹시 다른 곳에서 쓸 수도 있으니 유지, 필요 없으면 삭제 가능)
    double start_x = 0.0;
    double start_y = 0.0;
    double end_x = 0.0;
    double end_y = 0.0;
    bool is_initialized = false;
};

// =========================
// Global Variables
// =========================
extern std::map<int, Pose> cav_poses;
extern std::map<int, Pose> hv_poses;
extern std::map<int, std::vector<Pose>> cav_paths;
extern std::map<int, std::vector<Pose>> hv_paths;
extern std::map<int, HVState> hv_states;

// =========================
// 2개 ROI 정의 (CAV가 정지해야 하는 지점)
// =========================
extern std::map<int, Pose> cav_rois;

// HV 도착을 감지하는 지점
extern std::map<int, Pose> hv_rois;

// Yellow ROI zones (속도 감속 구간)
extern std::map<int, Pose> yellow_cav_rois;

// =========================
// Yellow Zone Groups (존별 속도 제어)
// =========================
extern std::map<int, int> yellow_roi_to_zone_group;

// Zone Group별 속도 설정
extern std::map<int, double> zone_group_speeds;

// CAV별 현재 속한 zone_group 추적 (0 = 아무 zone도 아님)
extern std::map<int, int> cav_current_zone_group;

// Key: ROI_ID, Value: 허가받은 차량 ID 목록
extern std::map<int, std::set<int>> roi_permit_vehicles;

extern double measured_hv_vel;

// ==========================================
// 측정할 두 지점 좌표 (원형 교차로 중심)
// ==========================================
extern const double CENTER_X;
extern const double CENTER_Y;

#endif  // MISSION_3_ROTARY_GLOBAL_HPP

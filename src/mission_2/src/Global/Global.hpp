/*
 * Mission 2 전체에서 공유되는 자료구조, 차선/Zone 상태, HV 협력 상태를 정의한다.
 * Planning/Control/Mission 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_2_GLOBAL_HPP
#define MISSION_2_GLOBAL_HPP

#include <rclcpp/rclcpp.hpp>

#include <deque>
#include <map>
#include <vector>

// =========================
// Path data structure
// =========================
struct PathPoint {
    double x;
    double y;
};

// =========================
// Controller state
// =========================
struct ControllerState {
    double speed_mps{0.5};
    double lookahead_m{0.3};
    double max_yaw_rate{1.5};
    double prev_x{0.0};
    double prev_y{0.0};
    double prev_yaw{0.0};
    bool has_prev{false};
    int red_flag{0};
};

struct CavState {
    int id;
    double start_x;
    double start_y;
    double start_yaw;
    int current_lap;
    bool is_initialized;
    bool is_in_line;
    bool is_finished;
};

// =========================
// HV Velocity Measurement (즉시 측정 방식)
// =========================
struct HVState {
    int id;

    // 즉각적인 속도 측정을 위한 변수
    bool is_first_msg = true;
    double prev_x = 0.0;
    double prev_y = 0.0;
    rclcpp::Time last_time;

    // 이동 평균 필터용 버퍼
    std::deque<double> vel_buffer;
    const size_t buffer_size = 10; // 최근 10개 평균
};

// =========================
// Global variables (차선/경로)
// =========================
extern std::vector<PathPoint> lane_paths[4];  // lane_paths[1], [2], [3] for lanes 1,2,3
extern std::map<int, int> lane_start_idx;     // lane_start_idx[1], [2], [3] = closest index per lane
extern std::vector<PathPoint> integrate_path_vector;  // Current active path
extern int current_lane;  // Current active lane (1=LEFT, 2=CENTER, 3=RIGHT)

// CAV pose
extern double cav_x;
extern double cav_y;
extern double cav_z;
extern double cav_yaw;

// HV positions (ID -> (x, y))
extern std::map<int, std::pair<double, double>> hv_positions;

// =========================
// Overlap Region Globals
// =========================
extern int overlap_start_idx; // 겹침 시작 인덱스 (Lane 3 기준)
extern int overlap_end_idx;   // 겹침 끝 인덱스 (Lane 3 기준)
extern bool overlap_detected;

// =========================
// Zone 1: HV/CAV ROI
// =========================
extern std::vector<PathPoint> hv_zone_polygon;
extern PathPoint cav_zone_start;
extern PathPoint cav_zone_end;
extern bool zone_collision_flag;
extern bool zone_cav_flag;  // CAV가 Zone 1 안에 있는지 여부

// =========================
// Zone 2
// =========================
extern PathPoint cav_zone2_start;
extern PathPoint cav_zone2_end;
extern PathPoint hv_zone2_start;
extern PathPoint hv_zone2_end;
extern bool zone2_collision_flag;

// =========================
// Zone 3 & 4 & 5
// =========================
extern PathPoint cav_zone3_start;
extern PathPoint cav_zone3_end;
extern bool zone3_collision_flag;

extern PathPoint cav_zone3_1_start;
extern PathPoint cav_zone3_1_end;
extern bool zone3_1_collision_flag;

extern PathPoint cav_zone3_2_start;
extern PathPoint cav_zone3_2_end;
extern bool zone3_2_collision_flag;

extern PathPoint cav_zone4_start;
extern PathPoint cav_zone4_end;
extern bool zone4_collision_flag;

extern PathPoint cav_zone5_start;
extern PathPoint cav_zone5_end;
extern bool zone5_collision_flag;

// =========================
// HV 속도 측정 상태
// =========================
extern std::map<int, HVState> hv_states;  // HV20, HV24 상태 저장
extern std::map<int, std::vector<PathPoint>> hv_paths;  // HV20, HV24 경로
extern double measured_hv20_vel;  // Lane 3 (HV20) 기본 속도
extern double measured_hv24_vel;  // Lane 2 (HV24) 기본 속도

#endif  // MISSION_2_GLOBAL_HPP

/*
 * Global.hpp에 선언된 차선/Zone/HV 협력 전역 상태의 정의.
 */
#include "Global/Global.hpp"

std::vector<PathPoint> lane_paths[4];
std::map<int, int> lane_start_idx;
std::vector<PathPoint> integrate_path_vector;
int current_lane = 2;

double cav_x = 0.0;
double cav_y = 0.0;
double cav_z = 0.0;
double cav_yaw = 0.0;

std::map<int, std::pair<double, double>> hv_positions;

int overlap_start_idx = -1;
int overlap_end_idx = -1;
bool overlap_detected = false;

// =========================
// Zone 1: HV ROI
// =========================
std::vector<PathPoint> hv_zone_polygon = {
    {3.5416667461395264, -1.8333686590194702},
    {3.537745475769043, -1.9098244905471802},
    {3.5260493755340576, -1.9686254262924194},
    {3.5105035305023193, -2.0161173343658447},
    {3.5028510093688965, -2.0345921516418457},
    {3.475167989730835, -2.08777117729187},
    {3.4365811347961426, -2.141998291015625},
    {3.4239115715026855, -2.1566786766052246},
    {3.3929789066314697, -2.1879186630249023},
    {3.333669900894165, -2.233895778656006},
    {3.24945068359375, -2.277170181274414},
    {3.1629221439361572, -2.301370143890381},
    {3.093331813812256, -2.3082242012023926},
    {3.020509958267212, -2.302333354949951}
};

// Zone 1: CAV ROI (선분)
PathPoint cav_zone_start = {4.5, -2.549999952316284};
PathPoint cav_zone_end   = {3.0, -2.549999952316284};

// Zone 1 flag
bool zone_collision_flag = false;
bool zone_cav_flag = false;  // CAV가 Zone 1 안에 있는지 여부

// =========================
// Zone 2
// =========================
// CAV Zone 2: 수직 선분
PathPoint cav_zone2_start = {5.058333396911621, 1.0199999809265137};
PathPoint cav_zone2_end = {5.058333396911621, 0.0};

// HV Zone 2: 수직 선분
PathPoint hv_zone2_start = {5.308333396911621, 0.4};
PathPoint hv_zone2_end = {5.308333396911621, -0.3};

// Zone 2 flag
bool zone2_collision_flag = false;

// =========================
// Zone 3 & 4 & 5
// =========================

// CAV Zone 3
PathPoint cav_zone3_start = {3.6, 2.55};
PathPoint cav_zone3_end = {4.41333333333333, 2.55};
bool zone3_collision_flag = false;

// CAV Zone 3_1
PathPoint cav_zone3_1_start = {2.68333333333333, 2.30833333333333};
PathPoint cav_zone3_1_end = {4.41333333333333, 2.30833333333333};
bool zone3_1_collision_flag = false;

// CAV Zone 3_2
PathPoint cav_zone3_2_start = {2.0, 2.55};
PathPoint cav_zone3_2_end = {3.6, 2.55};
bool zone3_2_collision_flag = false;

// CAV Zone 4
PathPoint cav_zone4_start = {5.05833333333333, 2.30833333333333};
PathPoint cav_zone4_end = {5.05833333333333, 1.74};
bool zone4_collision_flag = false;

// CAV Zone 5
PathPoint cav_zone5_start = {5.29166666666667, 2.30833333333333};
PathPoint cav_zone5_end = {5.29166666666667, -0.27};
bool zone5_collision_flag = false;

// =========================
// HV 속도 측정 상태
// =========================
std::map<int, HVState> hv_states;
std::map<int, std::vector<PathPoint>> hv_paths;
double measured_hv20_vel = 1.3;  // Lane 3 (HV20) 기본 속도
double measured_hv24_vel = 1.3;  // Lane 2 (HV24) 기본 속도

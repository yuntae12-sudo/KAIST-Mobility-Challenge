/*
 * Global.hpp에 선언된 ROI/Zone Group 전역 상태의 정의.
 */
#include "Global/Global.hpp"

std::map<int, Pose> cav_poses;
std::map<int, Pose> hv_poses;
std::map<int, std::vector<Pose>> cav_paths;
std::map<int, std::vector<Pose>> hv_paths;
std::map<int, HVState> hv_states;

// ======================
// 2개 ROI 정의 (반지름 0.075m)
// CAV ROI -> CAV HAVE TO STOP
// =========================
std::map<int, Pose> cav_rois = {
  {1, {1.43333333333, 1.5}},
  {2, {0, -0.4}}
//   {2, {0.1, -0.4}}
};

// hv
std::map<int, Pose> hv_rois = {
  {1, {0.9, 0.55}},
  {2, {1.1, -0.71}}
};

// Yellow ROI zones (속도 감속 구간)
std::map<int, Pose> yellow_cav_rois = {
  {1, {1.1945931911468506, 2.152200222015381}},     // CIRCLE 로타리 up ->DOWN  진입
  {2, {-0.4866666793823242, -0.53259406332969666}}, // CIRCLE 로타리 LEFT 진입
//   {3, {-3.738260269165039, -0.35879406332969666}},   // 2ND 사지교차로 LEFT 진입
  {3, {-3.518260269165039, -0.53259406332969666}},   // 2ND 사지교차로 LEFT 진입
  {4, {-2.2249999046325684, -1.2799999952316284}},   // 2ND 사지교차로 down-> up 진입
  {5, {-2.4662363529205322, 1.2755114316940308}},   // 2ND 사지교차로 RIGHT 진입
  {6, {-3.6093127727508545, 2.3087010383605957}},   // IST RIGHT 진입
  {7, {-1.1506917476654053, -2.30232834815979}},   // 3RD RIGHT 진입
  {8, {-0.4866666793823242, +0.53259406332969666}},
  {9, {-2.691666603088379, -1.2333333015441895}},
  {10, {-2.691666603088379, -1.5333333015441895}},
  {11, {-2.4662363529205322, 1.7755114316940308}}
};

// =========================
// Yellow Zone Groups (존별 속도 제어)
// =========================
// yellow_zone_group1: Zone 1,2 - 낮은 속도
// yellow_zone_group2: Zone 3,4,5,6,7 - 다른 속도
std::map<int, int> yellow_roi_to_zone_group = {
  {1, 1}, {3, 1}, {8, 1}, {9, 1}, {10, 1}, {11, 1},        // Zone 1,2 -> Group 1 // ->  zone3 append
  {2, 2}, {4, 2}, {5, 2}, {6, 2}, {7, 2}  // Zone 3,4,5,6,7,8 -> Group 2
};

// Zone Group별 속도 설정
std::map<int, double> zone_group_speeds = {
  {1, 0.8},  // Group 1 (Zone 1,2): 0.8 m/s
  {2, 0.8}   // Group 2 (Zone 3,4,5,6,7): 1.5 m/s
};

// CAV별 현재 속한 zone_group 추적 (0 = 아무 zone도 아님)
std::map<int, int> cav_current_zone_group = {
  {1, 0}, {2, 0}, {3, 0}, {4, 0}
};

// Key: ROI_ID, Value: 허가받은 차량 ID 목록
std::map<int, std::set<int>> roi_permit_vehicles;

double measured_hv_vel = 0.0;

// ==========================================
// 측정할 두 지점 좌표
// ==========================================
const double CENTER_X = 1.666666667;
const double CENTER_Y = 0.0;

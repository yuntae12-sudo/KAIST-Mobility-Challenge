/*
 * Global.hpp에 선언된 Zone/ROI/차량 위치 전역 상태의 정의.
 */
#include "Global/Global.hpp"

std::map<int, Pose> cav_poses;
std::map<int, std::vector<Pose>> cav_paths;
std::map<int, Pose> hv_poses;
std::map<int, std::vector<int>> prev_imminent_vehicles;

// =========================
// 2개 Zone 정의 (Zone 2, 4만)
// =========================
std::map<int, Pose> zones = {
  {2, {-2.333333333, 0.0}},
  {4, {2.001666784286499, 2.7}}
};

// =========================
// 4개 ROI 정의 (CAV1: ROI1,2 / CAV2: ROI3)
// =========================
std::map<int, Pose> rois = {
  {1, {1.1329693794250488, -0.5619539022445679}},  // CAV1이 먼저 지나는 ROI
  {2, {2.125271797180176, -0.6247305870056152}},   // CAV1이 두 번째로 지나는 ROI
  {3, {1.774999976158142, -1.8}},      // CAV2가 지나는 ROI
  {4, {5.05833333333333, 0.666666666666667}} // 합류 차선 roi
};

const double ROI_RADIUS = 0.23;       // ROI 1, 2, 3 (기존 크기)
const double ROI_MERGE_RADIUS = 0.6;  // ROI 4 (합류 구간용, 더 크게 설정)

// =========================
// CAV 색상 정의 (CAV 1,2만 사용)
// =========================
std::map<int, Color> cav_colors = {
  {1, {0.0, 0.0, 1.0, 0.8}},  // 파란색
  {2, {0.0, 1.0, 0.0, 0.8}}   // 초록색
};

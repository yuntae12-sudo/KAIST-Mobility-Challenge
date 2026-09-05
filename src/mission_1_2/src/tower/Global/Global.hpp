/*
 * Mission 1-2 Control Tower 노드 전체에서 공유되는 자료구조와 전역 상태(Zone/ROI/차량 위치)를 정의한다.
 * Planning/Tower/Visualizer 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_1_2_TOWER_GLOBAL_HPP
#define MISSION_1_2_TOWER_GLOBAL_HPP

#include <map>
#include <vector>

// =========================
// Pose Struct
// =========================
struct Pose {
  double x;
  double y;
};

// =========================
// CAV 색상 정의 (RGB)
// =========================
struct Color {
  double r, g, b, a;
};

// =========================
// Global Variables
// =========================
extern std::map<int, Pose> cav_poses;
extern std::map<int, std::vector<Pose>> cav_paths;
extern std::map<int, Pose> hv_poses;
extern std::map<int, std::vector<int>> prev_imminent_vehicles;

// =========================
// 2개 Zone 정의 (Zone 2, 4만)
// =========================
extern std::map<int, Pose> zones;

// =========================
// 4개 ROI 정의 (CAV1: ROI1,2 / CAV2: ROI3)
// =========================
extern std::map<int, Pose> rois;

extern const double ROI_RADIUS;       // ROI 1, 2, 3 (기존 크기)
extern const double ROI_MERGE_RADIUS; // ROI 4 (합류 구간용, 더 크게 설정)

// =========================
// CAV 색상 정의 (CAV 1,2만 사용)
// =========================
extern std::map<int, Color> cav_colors;

#endif  // MISSION_1_2_TOWER_GLOBAL_HPP

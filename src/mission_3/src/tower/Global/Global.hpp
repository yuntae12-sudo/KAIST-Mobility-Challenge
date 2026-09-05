/*
 * Mission 3 Control Tower 노드 전체에서 공유되는 자료구조와 전역 상태(Zone/ROI/차량 위치)를 정의한다.
 * Planning/Tower/Visualizer 알고리즘 구현은 포함하지 않는다.
 */
#ifndef MISSION_3_TOWER_GLOBAL_HPP
#define MISSION_3_TOWER_GLOBAL_HPP

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
// zone_id -> 지난 tick의 imminent 차량 목록(거리순)
extern std::map<int, std::vector<int>> prev_imminent_vehicles;

// =========================
// 3개 Zone 정의 (Zone 4는 제거됨)
// =========================
extern std::map<int, Pose> zones;

// =========================
// 4개 ROI 정의 (반지름 0.075m)
// =========================
extern std::map<int, Pose> rois;

extern const double ROI_RADIUS;  // ROI 반지름

// =========================
// CAV 색상 정의
// =========================
extern std::map<int, Color> cav_colors;

#endif  // MISSION_3_TOWER_GLOBAL_HPP

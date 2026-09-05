/*
 * 특정 도메인(Planning/Rotary/Visualizer)에 종속되지 않는 범용 헬퍼 함수를 모은다.
 */
#ifndef MISSION_3_ROTARY_UTILS_HPP
#define MISSION_3_ROTARY_UTILS_HPP

#include <string>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Distance / Geometry
// ==============================
double calculate_distance(Pose pose1, Pose pose2);
double GetArcLength(double x1, double y1, double x2, double y2, double cx, double cy);

// ==============================
// Path CSV
// ==============================
std::vector<Pose> load_csv_file(const std::string& file_path);

#endif  // MISSION_3_ROTARY_UTILS_HPP

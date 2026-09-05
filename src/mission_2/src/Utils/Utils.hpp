/*
 * 특정 도메인(Planning/Control/Mission)에 종속되지 않는 범용 헬퍼 함수를 모은다.
 */
#ifndef MISSION_2_UTILS_HPP
#define MISSION_2_UTILS_HPP

#include <geometry_msgs/msg/quaternion.hpp>

#include <string>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// CAV ID
// ==============================
std::string twoDigitId(int id);

// ==============================
// Distance / Angle
// ==============================
double calculateDistance(double x1, double y1, double x2, double y2);
double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw);
double normalizeAngle(double angle);

// ==============================
// Path CSV
// ==============================
bool loadPathCsv(const std::string& csv_path, std::vector<PathPoint>& out);

#endif  // MISSION_2_UTILS_HPP

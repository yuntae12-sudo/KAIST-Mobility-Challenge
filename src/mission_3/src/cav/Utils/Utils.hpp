/*
 * 특정 도메인(Planning/Control/Mission)에 종속되지 않는 범용 헬퍼 함수를 모은다.
 */
#ifndef MISSION_3_CAV_UTILS_HPP
#define MISSION_3_CAV_UTILS_HPP

#include <geometry_msgs/msg/quaternion.hpp>

#include <string>
#include <vector>

#include "Global/Global.hpp"

// ==============================
// Quaternion / Angle
// ==============================
double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw);
double normalizeAngle(double angle);

// ==============================
// CAV ID
// ==============================
int readCavIdFromEnvOrDefault(int default_id);
std::string twoDigitId(int id);

// ==============================
// Path CSV
// ==============================
bool loadPathCsv(const std::string& csv_path, std::vector<integrate_path_struct>& out);

#endif  // MISSION_3_CAV_UTILS_HPP

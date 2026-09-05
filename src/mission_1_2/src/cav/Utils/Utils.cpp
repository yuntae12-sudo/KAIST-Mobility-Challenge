/*
 * Utils.hpp에 선언된 범용 헬퍼 함수들의 구현.
 */
#include "Utils/Utils.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <sstream>

// 쿼터니언으로부터 yaw 각도를 계산한다. 쿼터니언이 비정상(거의 0)이면 이전 yaw를 그대로 반환한다.
double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw)
{
  const double norm2 = qmsg.x*qmsg.x + qmsg.y*qmsg.y + qmsg.z*qmsg.z + qmsg.w*qmsg.w;
  if (norm2 <= 1e-6) return fallback_yaw;

  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// 각도를 -pi ~ pi 범위로 정규화한다.
double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// 환경변수 CAV_ID가 설정되어 있으면 그 값을, 아니면 default_id를 반환한다.
int readCavIdFromEnvOrDefault(int default_id)
{
  const char* s = std::getenv("CAV_ID");
  if (!s || std::string(s).empty()) return default_id;
  try { return std::stoi(s); } catch (...) { return default_id; }
}

// CAV ID를 두 자리 문자열(01~99)로 변환한다.
std::string twoDigitId(int id)
{
  if (id < 0) id = 0;
  if (id > 99) id = 99;

  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << id;
  return oss.str();
}

// x,y 좌표 쌍으로 이루어진 경로 CSV 파일을 읽어 온다.
bool loadPathCsv(const std::string& csv_path, std::vector<integrate_path_struct>& out)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) return false;

  out.clear();
  std::string line;
  if (std::getline(file, line)) {
    std::stringstream ss(line); double x, y; char comma;
    if (!(ss >> x >> comma >> y)) { } else { out.push_back({x, y}); }
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line); double x, y; char comma;
    if (!(ss >> x)) continue; if (!(ss >> comma)) continue; if (!(ss >> y)) continue;
    if (std::isnan(x) || std::isnan(y)) continue;
    out.push_back({x, y});
  }

  return out.size() >= 2;
}

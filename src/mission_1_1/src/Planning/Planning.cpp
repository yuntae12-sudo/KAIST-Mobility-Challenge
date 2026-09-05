/*
 * Planning.hpp에 선언된 경로 탐색/Lookahead 계산 함수들의 구현.
 */
#include "Planning/Planning.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "Utils/Utils.hpp"

// Closest Point 탐색과 커브 판단을 순서대로 수행한다.
void PlanningProcess(const std::vector<integrate_path_struct>& path,
                      double x_m, double y_m,
                      const ControllerState& st,
                      int& closest_idx,
                      bool& corner_detected)
{
  closest_idx = findClosestPoint(path, x_m, y_m);
  corner_detected = isCorner(path, st.lookahead_m, closest_idx);
}

// 목표 속도 반영 후 Lookahead를 갱신하고, 그 거리 이상 떨어진 목표 Waypoint를 찾는다.
int FindTargetWaypoint(const std::vector<integrate_path_struct>& path,
                        double x_m, double y_m,
                        ControllerState& st)
{
  GetLd(st);
  return findWaypoint(path, x_m, y_m, st.lookahead_m);
}

// 현재 위치와 가장 가까운 Global Path 인덱스를 탐색한다.
// 모든 경로점과의 거리를 비교하여 최소 거리 지점을 반환한다.
int findClosestPoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m)
{
  if (path.empty()) return -1;
  double min_d = -1.0; int best = 0;

  for (size_t i = 0; i < path.size(); i++) {
    const double dx = path[i].x - x_m;
    const double dy = path[i].y - y_m;
    const double d  = std::hypot(dx, dy);

    if (min_d < 0.0 || d < min_d) {
      min_d = d;
      best = static_cast<int>(i);
    }
  }
  closest_index = best;
  return best;
}

// Closest Point 이후에서 Lookahead 조건을 만족하는 목표점을 탐색한다.
int findWaypoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m, double L_d) {
  if (path.empty()) { std::cout << "Path Empty" << std::endl; return -1; }
  int closest_idx = findClosestPoint(path, x_m, y_m);
  for (int i = closest_idx; i < (int)path.size(); ++i) {
    double d = hypot(path[i].x - x_m, path[i].y - y_m);
    if (d > L_d) return i;
  }
  return (int)path.size() - 1;
}

// 향후 20개 앞선 지점과 현재 지점의 진행 방향 차이로 커브 여부를 판단한다.
bool isCorner(const std::vector<integrate_path_struct>& path, double /*L_d*/, int closest_idx) {
  int future_offset = 20;
  int path_size = (int)path.size();
  if (closest_idx < 0) return false;
  if (closest_idx + future_offset + 1 >= path_size) return false;

  double cur_ang = atan2(path[closest_idx + 1].y - path[closest_idx].y,
                         path[closest_idx + 1].x - path[closest_idx].x);

  int idx_future_start = closest_idx + future_offset;
  double fut_ang = atan2(path[idx_future_start + 1].y - path[idx_future_start].y,
                         path[idx_future_start + 1].x - path[idx_future_start].x);

  double diff = fabs(normalizeAngle(fut_ang - cur_ang));
  if (diff > M_PI / 2) diff = M_PI - diff;

  double threshold_deg = 10.0;
  double threshold_rad = threshold_deg * (M_PI / 180.0);
  return (diff > threshold_rad);
}

// 현재 속도를 기반으로 Pure Pursuit의 Lookahead 거리를 동적으로 계산한다.
void GetLd(ControllerState& st) {
  double gain_ld = 0.4;
  double max_ld  = 0.355;
  double min_ld  = 0.15;

  double velocity = st.speed_mps;
  double ld = gain_ld * velocity;
  st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
}

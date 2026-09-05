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
// 순환 경로이므로 경로 끝 이후 처음 인덱스부터 다시 탐색한다.
int findWaypoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m, double L_d) {
    if (path.empty()) { std::cout << "Path Empty" << std::endl; return -1; }

    int closest_idx = findClosestPoint(path, x_m, y_m);
    int path_size = (int)path.size();

    // Loop를 통해 순환 탐색 (최대 전체 경로 길이만큼 확인)
    for (int i = 0; i < path_size; ++i) {
        // [핵심 변경] 모듈러 연산(%)을 사용하여 인덱스가 끝을 넘어가면 0으로 돌아오게 함
        int target_path_idx_with_LD = (closest_idx + i) % path_size;

        double d = hypot(path[target_path_idx_with_LD].x - x_m, path[target_path_idx_with_LD].y - y_m);
        if (d > L_d) {
            return target_path_idx_with_LD;
        }
    }

    // 혹시라도 못 찾으면 현재 위치 바로 다음 점 반환 (순환 고려)
    return (closest_idx + 1) % path_size;
}

// 향후 60개 앞선 지점과 현재 지점의 진행 방향 차이로 커브 여부를 판단한다.
bool isCorner(const std::vector<integrate_path_struct>& path, double /*L_d*/, int closest_idx) {
    int future_offset = 60;
    int path_size = (int)path.size();
    if (closest_idx + future_offset + 1 >= path_size) { return false; }

    double cur_ang = atan2(path[closest_idx + 1].y - path[closest_idx].y,
                           path[closest_idx + 1].x - path[closest_idx].x);
    int idx_future_start = closest_idx + future_offset;
    double fut_ang = atan2(path[idx_future_start + 1].y - path[idx_future_start].y,
                           path[idx_future_start + 1].x - path[idx_future_start].x);

    double diff = fabs(normalizeAngle(fut_ang - cur_ang));
    if (diff > M_PI / 2) { diff = M_PI - diff; }

    double threshold_deg = 10.0;
    double threshold_rad = threshold_deg * (M_PI / 180.0);
    return (diff > threshold_rad);
}

// Pure Pursuit의 Lookahead 거리를 계산한다. ld는 속도와 무관하게 3.0으로 고정되어 있어
// 항상 max_ld(0.4)로 clamp된다 (사실상 고정 Lookahead로 동작).
void GetLd(ControllerState& st) {
  double max_ld  = 0.4;
  double min_ld  = 0.3;
  double ld = 3.0;
  st.lookahead_m = std::max(min_ld, std::min(max_ld, ld));
}

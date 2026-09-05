/*
 * Planning.hpp에 선언된 경로 탐색/Zone Group 판단 함수들의 구현.
 */
#include "Planning/Planning.hpp"

#include <limits>

#include "Utils/Utils.hpp"

// 현재 위치에서 가장 가까운 웨이포인트 인덱스를 찾는다.
int find_closest_waypoint_index(const std::vector<Pose>& path, Pose current_pose) {
  if (path.empty()) return -1;

  double min_distance = std::numeric_limits<double>::max();
  int closest_idx = 0;

  for (size_t i = 0; i < path.size(); i++) {
    const double distance = calculate_distance(path[i], current_pose);
    if (distance < min_distance) {
      min_distance = distance;
      closest_idx = static_cast<int>(i);
    }
  }
  return closest_idx;
}

// CAV의 현재 zone group 판단 함수
int get_zone_group_for_cav(int /*cav_index*/, const Pose& cav_pose, double detection_radius) {
    for (const auto& [yellow_roi_id, yellow_roi_pose] : yellow_cav_rois) {
        double dist = calculate_distance(cav_pose, yellow_roi_pose);
        if (dist <= detection_radius) {
            // yellow_roi_to_zone_group에서 해당 zone group 반환
            return yellow_roi_to_zone_group[yellow_roi_id];
        }
    }
    return 0;  // 어떤 zone에도 속하지 않음
}

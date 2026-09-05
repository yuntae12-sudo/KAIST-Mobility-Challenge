/*
 * Planning.hpp에 선언된 경로 탐색/겹침 판단/Sub CAV 선정 함수들의 구현.
 */
#include "Planning/Planning.hpp"

#include <algorithm>
#include <iostream>
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

// 현재 인덱스부터 lookahead_count만큼의 웨이포인트를 추출한다.
std::vector<Pose> get_lookahead_waypoints(
    const std::vector<Pose>& path,
    int current_idx,
    int lookahead_count) {

  std::vector<Pose> lookahead_points;

  if (path.empty() || current_idx < 0) {
    return lookahead_points;
  }

  int end_idx = std::min(current_idx + lookahead_count, static_cast<int>(path.size()));

  for (int i = current_idx; i < end_idx; i++) {
    lookahead_points.push_back(path[i]);
  }

  return lookahead_points;
}

// 두 CAV의 Lookahead 구간이 겹치는지 판단한다.
bool check_path_overlap(
    const std::vector<Pose>& main_lookahead,
    const std::vector<Pose>& sub_lookahead,
    double overlap_threshold,
    double& csv_distance) {

  if (main_lookahead.empty() || sub_lookahead.empty()) {
    return false;
  }

  double min_dist_found = std::numeric_limits<double>::max();
  bool is_overlapping = false;

  for (const auto& main_point : main_lookahead) {
    for (const auto& sub_point : sub_lookahead) {
      double dist = calculate_distance(main_point, sub_point);

      if (dist < min_dist_found) {
        min_dist_found = dist;
      }

      if (dist < overlap_threshold) {
        is_overlapping = true;
      }
    }
  }

  csv_distance = min_dist_found;

  if (min_dist_found < overlap_threshold) {
    std::cout << "[PATH DEBUG] Min Distance: " << min_dist_found
              << "m (Overlap: " << (is_overlapping ? "YES" : "NO") << ")" << std::endl;
  }

  return is_overlapping;
}

// 차량이 Zone 중심에서 멀어지고 있는지 판단한다.
bool is_moving_away(int cav_id, Pose zone_origin, const std::vector<Pose>& path) {
    if (path.empty()) return false;

    int current_idx = find_closest_waypoint_index(path, cav_poses[cav_id]);
    int future_idx = std::min(current_idx + 30, static_cast<int>(path.size()) - 1);

    if (current_idx >= static_cast<int>(path.size()) - 5) return true;

    Pose current_pose = path[current_idx];
    Pose future_pose = path[future_idx];

    double dist_current = calculate_distance(current_pose, zone_origin);
    double dist_future = calculate_distance(future_pose, zone_origin);

    return dist_future > dist_current;
}

// 2대 차량용 간소화된 Sub CAV 선택 함수.
int select_sub_cav_for_two_vehicles(
    int main_cav_id,
    const std::vector<int>& precollision_candidates,
    Pose zone_origin,
    double overlap_threshold,
    int lookahead_distance) {

  if (precollision_candidates.empty()) {
    return -1;
  }

  if (precollision_candidates.size() == 1) {
    int sub_id = precollision_candidates[0];

    if (cav_paths[main_cav_id].empty() || cav_paths[sub_id].empty()) {
      return -1;
    }

    int main_idx = find_closest_waypoint_index(cav_paths[main_cav_id], cav_poses[main_cav_id]);
    int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);

    std::vector<Pose> main_lookahead = get_lookahead_waypoints(cav_paths[main_cav_id], main_idx, lookahead_distance);
    std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);

    double csv_distance = 0.0;
    bool overlaps = check_path_overlap(main_lookahead, sub_lookahead, overlap_threshold, csv_distance);

    if (!overlaps) {
      std::cout << "[SUB_CAV] CAV_" << sub_id << " selected (no overlap)" << std::endl;
      return sub_id;
    }

    // 경로 겹침이 있으면 안전 여부 체크
    int main_overlap_idx = -1;
    int sub_overlap_idx = -1;
    double min_overlap_dist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < main_lookahead.size(); i++) {
      for (size_t j = 0; j < sub_lookahead.size(); j++) {
        double dist = calculate_distance(main_lookahead[i], sub_lookahead[j]);
        if (dist < overlap_threshold && dist < min_overlap_dist) {
          min_overlap_dist = dist;
          main_overlap_idx = static_cast<int>(i);
          sub_overlap_idx = static_cast<int>(j);
        }
      }
    }

    if (main_overlap_idx != -1 && sub_overlap_idx != -1) {
      int idx_diff = abs(main_overlap_idx - sub_overlap_idx);

      std::cout << "[OVERLAP CHECK] Main_idx: " << main_overlap_idx
                << ", Sub_idx: " << sub_overlap_idx
                << ", Diff: " << idx_diff << std::endl;

      if (idx_diff > 150 || (main_overlap_idx > 170 && sub_overlap_idx < 30)) {
        std::cout << "[SAFE OVERLAP] CAV_" << sub_id << " approved" << std::endl;
        return sub_id;
      }
    }

    std::cout << "[UNSAFE OVERLAP] CAV_" << sub_id << " rejected" << std::endl;
    return -1;
  }

  return -1;
}

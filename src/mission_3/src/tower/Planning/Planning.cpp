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

  return is_overlapping;
}

// 차량이 Zone 중심에서 멀어지고 있는지 판단한다.
bool is_moving_away(int cav_id, Pose zone_origin, const std::vector<Pose>& path) {
    if (path.empty()) return false;

    // 1. 현재 위치의 인덱스 찾기
    int current_idx = find_closest_waypoint_index(path, cav_poses[cav_id]);

    // 2. 미래 위치 가져오기
    int future_idx = std::min(current_idx + 30, static_cast<int>(path.size()) - 1);

    // 경로 끝에 도달했으면 멀어지는 것으로 간주
    if (current_idx >= static_cast<int>(path.size()) - 5) return true;

    Pose current_pose = path[current_idx];
    Pose future_pose = path[future_idx];

    // 3. 거리 비교
    double dist_current = calculate_distance(current_pose, zone_origin);
    double dist_future = calculate_distance(future_pose, zone_origin);

    // 미래의 거리가 현재 거리보다 크면 멀어지는 중이런식으로..
    return dist_future > dist_current;
}

// Main CAV와 Precollision 후보군에서 Sub CAV 선택
int select_best_sub_cav(int main_cav_id, const std::vector<int>& precollision_candidates, Pose zone_origin, double overlap_threshold, int lookahead_distance) {
  // 방어 로직: 빈 경로나 없는 CAV 체크
  if (cav_paths.find(main_cav_id) == cav_paths.end() || cav_paths[main_cav_id].empty()) {
    return -1;
  }
  if (cav_poses.find(main_cav_id) == cav_poses.end()) {
    return -1;
  }

  int main_idx = find_closest_waypoint_index(cav_paths[main_cav_id], cav_poses[main_cav_id]);
  std::vector<Pose> main_lookahead = get_lookahead_waypoints(cav_paths[main_cav_id], main_idx, lookahead_distance);

  std::vector<int> non_overlapping_candidates;
  for (int sub_id : precollision_candidates) {
    // 각 sub_id도 체크
    if (cav_paths.find(sub_id) == cav_paths.end() || cav_paths[sub_id].empty()) continue;
    if (cav_poses.find(sub_id) == cav_poses.end()) continue;

    int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);
    std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);

    double csv_distance = 0.0;
    if (!check_path_overlap(main_lookahead, sub_lookahead, overlap_threshold, csv_distance)) {
      non_overlapping_candidates.push_back(sub_id);
    }
  }

  // 경로 안 겹치는 후보군이 없을 때, 안전한 경로 겹침 체크
  if (non_overlapping_candidates.empty()) {
    std::vector<int> safe_overlap_candidates;

    for (int sub_id : precollision_candidates) {
      int sub_idx = find_closest_waypoint_index(cav_paths[sub_id], cav_poses[sub_id]);
      std::vector<Pose> sub_lookahead = get_lookahead_waypoints(cav_paths[sub_id], sub_idx, lookahead_distance);

      // 겹침 인덱스 찾기
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

      // 겹침이 있으면 인덱스 차이 계산
      if (main_overlap_idx != -1 && sub_overlap_idx != -1) {
        int idx_diff = abs(main_overlap_idx - sub_overlap_idx);

        // Main의 앞쪽 점(큰 인덱스)과 Sub의 뒤쪽 점(작은 인덱스)이 만나면 안전
        // 또는 인덱스 차이가 충분히 크면 시간 여유가 있음
        if (idx_diff > 250 || (main_overlap_idx > 200 && sub_overlap_idx < 20)) {
          safe_overlap_candidates.push_back(sub_id);
        }
      }
    }

    // Safe overlap 후보군이 있으면 그 중에서 가장 가까운 차량 선택
    if (!safe_overlap_candidates.empty()) {
      int selected_sub = safe_overlap_candidates[0];
      double min_d = calculate_distance(cav_poses[selected_sub], zone_origin);
      for (int id : safe_overlap_candidates) {
        double d = calculate_distance(cav_poses[id], zone_origin);
        if (d < min_d) { min_d = d; selected_sub = id; }
      }
      return selected_sub;
    }

    // 안전한 경로 겹침도 없으면 -1 반환 (Main만 진행)
    return -1;
  }

  int selected_sub = non_overlapping_candidates[0];
  double min_d = calculate_distance(cav_poses[selected_sub], zone_origin);
  for (int id : non_overlapping_candidates) {
    double d = calculate_distance(cav_poses[id], zone_origin);
    if (d < min_d) { min_d = d; selected_sub = id; }
  }
  return selected_sub;
}

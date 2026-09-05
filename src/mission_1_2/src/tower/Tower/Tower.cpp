/*
 * Tower.hpp에 선언된 Zone/ROI 판단과 RED_FLAG 발행 로직의 구현.
 */
#include "Tower/Tower.hpp"

#include <algorithm>
#include <iostream>

#include "Planning/Planning.hpp"
#include "Utils/Utils.hpp"

// =========================
// Tower Process: ROI 기반 CAV1/CAV2 제어 판단 후 Zone 2, 4를 모니터링한다.
// =========================
void TowerProcess(
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance) {
  // ROI 기반 CAV2 제어 (실시간 위치 기반)
  bool cav2_should_stop = should_cav2_stop_by_roi(node);

  static bool prev_cav2_roi_stop = false;
  if (cav2_should_stop != prev_cav2_roi_stop) {
    auto msg = std_msgs::msg::Int32();
    msg.data = cav2_should_stop ? 1 : 0;
    red_flag_pubs[2]->publish(msg);

    if (cav2_should_stop) {
      RCLCPP_ERROR(node->get_logger(), "[ROI CONTROL] CAV_2 RED_FLAG by ROI logic");
    } else {
      RCLCPP_INFO(node->get_logger(), "[ROI CONTROL] CAV_2 GREEN_FLAG by ROI logic");
    }

    prev_cav2_roi_stop = cav2_should_stop;
  }

  // ---------------------------------------------------------
  // ROI 4 기반 CAV1 제어 (합류 구간)
  // ---------------------------------------------------------
  bool cav1_merge_stop = should_cav1_stop_by_merge_roi(node);
  static bool prev_cav1_merge_stop = false;

  // 상태가 변했을 때만 Publish (토픽 부하 방지)
  if (cav1_merge_stop != prev_cav1_merge_stop) {
      auto msg = std_msgs::msg::Int32();
      msg.data = cav1_merge_stop ? 1 : 0;
      red_flag_pubs[1]->publish(msg); // CAV1에게 정지 신호 보냄

      if (cav1_merge_stop) {
          RCLCPP_ERROR(node->get_logger(), "[MERGE CONTROL] CAV_1 RED_FLAG! (Conflict at ROI 4)");
      } else {
          RCLCPP_INFO(node->get_logger(), "[MERGE CONTROL] CAV_1 GREEN_FLAG (ROI 4 Clear)");
      }

      prev_cav1_merge_stop = cav1_merge_stop;
  }

  // Zone 2, 4만 모니터링
  monitor_zone(2, node, red_flag_pubs, prev_red_flag_vehicles,
               precollision_radius, imminent_collision_radius,
               overlap_threshold, lookahead_distance);
  monitor_zone(4, node, red_flag_pubs, prev_red_flag_vehicles,
               precollision_radius, imminent_collision_radius,
               overlap_threshold, lookahead_distance);
}

// =========================
// Precollision Zone 체크
// =========================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// Imminent Collision Zone 체크
// =========================
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// ROI 체크 함수
// =========================
static bool is_in_roi(Pose cav_pose, Pose roi_origin, double radius) {
    double distance = calculate_distance(cav_pose, roi_origin);
    return distance <= radius;
}

// =========================
// ROI 기반 CAV2 제어 로직 (상태 유지 방식 적용)
// =========================
bool should_cav2_stop_by_roi(std::shared_ptr<rclcpp::Node> node) {
  // 차량 존재 확인
  if (cav_poses.find(1) == cav_poses.end() || cav_poses.find(2) == cav_poses.end()) {
    return false;
  }

  Pose cav1_pose = cav_poses[1];
  Pose cav2_pose = cav_poses[2];

  // 현재 각 ROI까지의 거리 계산
  double dist_cav1_to_roi1 = calculate_distance(cav1_pose, rois[1]);
  double dist_cav1_to_roi2 = calculate_distance(cav1_pose, rois[2]);

  // CAV2가 ROI3(진입 대기 구역)에 있는지 확인
  bool cav2_in_roi3 = is_in_roi(cav2_pose, rois[3], ROI_RADIUS);

  // ==========================================
  // [수정된 핵심 로직] 상태 기억 (Latching)
  // ==========================================
  // static 변수를 사용하여 함수가 종료되어도 값을 기억하게 함
  // true: CAV1이 ROI 1을 지났고 아직 ROI 2를 안 지남 (교차로 점유 중)
  // false: 교차로 비어있음
  static bool cav1_is_crossing = false;

  // 1. 진입 감지: CAV1이 ROI 1에 들어왔다면 -> 교차로 점유 시작 (Lock)
  if (dist_cav1_to_roi1 <= ROI_RADIUS) {
    if (!cav1_is_crossing) {
        RCLCPP_WARN(node->get_logger(), "[LATCH] CAV1 Entered ROI 1 -> CROSSING START");
        cav1_is_crossing = true;
    }
  }

  // 2. 탈출 감지: CAV1이 ROI 2에 도달했다면 -> 교차로 점유 해제 (Unlock)
  // ROI 2에 "도달"했을 때 해제하거나, 더 안전하게 하려면 "지나갔을 때" 해제해도 됩니다.
  // 여기서는 ROI 2 반경 안에 들어오면 안전하다고 판단하여 해제합니다.
  if (dist_cav1_to_roi2 <= ROI_RADIUS) {
    if (cav1_is_crossing) {
        RCLCPP_INFO(node->get_logger(), "[LATCH] CAV1 Reached ROI 2 -> CROSSING END");
        cav1_is_crossing = false;
    }
  }

  // 디버그 로그 (상태 확인용)
  static int debug_cnt = 0;
  if (debug_cnt++ % 25 == 0) {
    RCLCPP_INFO(node->get_logger(),
                "[ROI STATE] C1_Crossing: %s | C2_in_ROI3: %s | D1: %.2f, D2: %.2f",
                cav1_is_crossing ? "YES (BUSY)" : "NO (FREE)",
                cav2_in_roi3 ? "YES" : "NO",
                dist_cav1_to_roi1, dist_cav1_to_roi2);
  }

  // 3. 최종 판단
  // CAV2가 대기 구역(ROI 3)에 있고 && CAV1이 교차로를 건너는 중이라면 -> 정지
  if (cav2_in_roi3 && cav1_is_crossing) {
    return true; // RED FLAG
  }

  return false; // GREEN FLAG
}

// =========================
// ROI 4 (합류 구간) 기반 CAV1 제어 로직 (반지름 확대 적용)
// =========================
bool should_cav1_stop_by_merge_roi(std::shared_ptr<rclcpp::Node> node) {
  if (cav_poses.find(1) == cav_poses.end() || cav_poses.find(2) == cav_poses.end()) {
    return false;
  }

  // ROI 4(합류 구간)는 다른 ROI보다 넓은 ROI_MERGE_RADIUS로 판정한다.
  bool cav1_in_merge = is_in_roi(cav_poses[1], rois[4], ROI_MERGE_RADIUS);
  bool cav2_in_merge = is_in_roi(cav_poses[2], rois[4], ROI_MERGE_RADIUS);

  static bool is_conflict_state = false;

  // [LOCK] 두 차량이 큰 반경 안에 들어오면
  if (cav1_in_merge && cav2_in_merge) {
    if (!is_conflict_state) {
        RCLCPP_WARN(node->get_logger(), "[MERGE START] Both Vehicles in ROI 4 (Large Zone) -> LOCK");
        is_conflict_state = true;
    }
  }

  // [UNLOCK] CAV 2가 큰 반경을 완전히 벗어나면
  if (is_conflict_state && !cav2_in_merge) {
    RCLCPP_INFO(node->get_logger(), "[MERGE END] CAV 2 Left ROI 4 -> UNLOCK");
    is_conflict_state = false;
  }

  return is_conflict_state;
}

// =========================
// Zone 모니터링 및 제어 함수
// =========================
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance) {

  Pose zone_origin = zones[zone_id];
  std::vector<int> precollision_vehicles;
  std::vector<std::pair<double, int>> imminent_with_dist;
  int cav_sub = -1;
  int main_cav_id = -1;

  for (const auto& [cav_id, pose] : cav_poses) {
    double dist = calculate_distance(pose, zone_origin);

    if (dist <= precollision_radius) {
      precollision_vehicles.push_back(cav_id);
    }
    if (dist <= imminent_collision_radius) {
      imminent_with_dist.push_back({dist, cav_id});
    }
  }

  std::sort(imminent_with_dist.begin(), imminent_with_dist.end());

  std::vector<int> imminent_vehicles;
  for (const auto& p : imminent_with_dist) {
    imminent_vehicles.push_back(p.second);
  }

  if (imminent_vehicles != prev_imminent_vehicles[zone_id]) {
    std::string imminent_ids;
    for (int id : imminent_vehicles) imminent_ids += std::to_string(id) + " ";

    if (!imminent_vehicles.empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "[ZONE_%d IMMINENT] CAVs (Closest First): %s",
                  zone_id, imminent_ids.c_str());
    } else {
      RCLCPP_INFO(node->get_logger(),
                  "[ZONE_%d IMMINENT] EMPTY",
                  zone_id);
    }

    prev_imminent_vehicles[zone_id] = imminent_vehicles;
  }

  std::vector<int> current_red_flag_vehicles;

  if (imminent_vehicles.empty()) {
    // Imminent 차량 없음
  } else {
    main_cav_id = imminent_vehicles[0];
    RCLCPP_INFO(node->get_logger(), "[ZONE_%d MAIN_CAV] Selected Closest CAV_%d (Dist: %.2fm)",
                zone_id, main_cav_id, imminent_with_dist[0].first);

    if (!precollision_vehicles.empty()) {
      std::vector<int> sub_candidates;
      for (int cav_id : precollision_vehicles) {
        if (cav_id == main_cav_id) continue;
        if (is_moving_away(cav_id, zone_origin, cav_paths[cav_id])) continue;
        sub_candidates.push_back(cav_id);
      }

      if (!sub_candidates.empty()) {
        cav_sub = select_sub_cav_for_two_vehicles(
            main_cav_id, sub_candidates, zone_origin,
            overlap_threshold, lookahead_distance);

        if (cav_sub != -1) {
          RCLCPP_INFO(node->get_logger(), "[ZONE_%d SUB_CAV] Selected CAV_%d", zone_id, cav_sub);
        } else {
          RCLCPP_ERROR(node->get_logger(), "[ZONE_%d] Candidate overlaps! No Sub CAV selected.", zone_id);
        }

        for (int cav_id : sub_candidates) {
          if (cav_id != cav_sub) {
            current_red_flag_vehicles.push_back(cav_id);
          }
        }
      }
    }
  }

  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG STATE CHANGED]", zone_id);

    for (int prev_id : prev_red_flag_vehicles[zone_id]) {
      if (std::find(current_red_flag_vehicles.begin(), current_red_flag_vehicles.end(), prev_id)
          == current_red_flag_vehicles.end()) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 0;
        red_flag_pubs[prev_id]->publish(msg);
        RCLCPP_INFO(node->get_logger(), "[ZONE_%d GREEN_FLAG] CAV_%d -> RESUME", zone_id, prev_id);
      }
    }

    for (int cav_id : current_red_flag_vehicles) {
      auto msg = std_msgs::msg::Int32();
      msg.data = 1;
      red_flag_pubs[cav_id]->publish(msg);
      RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG] CAV_%d -> STOP (Main=CAV_%d, Sub=CAV_%d)",
                  zone_id, cav_id, main_cav_id, cav_sub);
    }

    prev_red_flag_vehicles[zone_id] = current_red_flag_vehicles;
  }
}

/*
 * Tower.hpp에 선언된 Zone 판단과 RED_FLAG 발행 로직의 구현.
 */
#include "Tower/Tower.hpp"

#include <algorithm>
#include <iostream>

#include "Planning/Planning.hpp"
#include "Utils/Utils.hpp"

// =========================
// Precollision Zone 체크 (반지름 0.7m)
// =========================
bool is_in_precollision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// Imminent Collision Zone 체크 (반지름 0.5m)
// =========================
bool is_in_imminent_collision_zone(Pose cav_pose, Pose zone_origin, double radius) {
    double distance = calculate_distance(cav_pose, zone_origin);
    return distance <= radius;
}

// =========================
// Zone 모니터링 및 RED/GREEN_FLAG 제어 함수
// =========================
void monitor_zone(
    int zone_id,
    std::shared_ptr<rclcpp::Node> node,
    std::map<int, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr>& red_flag_pubs,
    std::map<int, std::vector<int>>& prev_red_flag_vehicles,
    double precollision_radius,
    double imminent_collision_radius,
    double overlap_threshold,
    int lookahead_distance,
    const std::map<int, int>& cav_index_map_actual_to_idx,
    const std::map<int, int>& cav_index_map_idx_to_actual) {

  Pose zone_origin = zones[zone_id];
  std::vector<int> precollision_vehicles;

  std::vector<std::pair<double, int>> imminent_with_dist; // first는 거리, second는 차 id
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
 //지금 이 Zone에 들어온 Imminent 차량 목록이, 지난번에 비해 바뀌었을 때만 로그 찍는 부분 //리스트의 순서나 구성이 바뀌면 로그출력
std::sort(imminent_with_dist.begin(), imminent_with_dist.end()); //정렬

std::vector<int> imminent_vehicles;
for (const auto& p : imminent_with_dist) {
  imminent_vehicles.push_back(p.second);
}

// 이전 imminent랑 비교해야 함
if (imminent_vehicles != prev_imminent_vehicles[zone_id]) { //prev_imminent_vehicles가 map<int, vector<int>> key: zone_id , value: 그 zone의 “이전 imminent 리스트”
  std::string imminent_ids;
  for (int id : imminent_vehicles) imminent_ids += std::to_string(id) + " ";

  if (!imminent_vehicles.empty()) {
    // RCLCPP_WARN(node->get_logger(),
    //             "[ZONE_%d IMMINENT] CAVs (Closest First): %s",
    //             zone_id, imminent_ids.c_str());
  } else {
    // RCLCPP_INFO(node->get_logger(),
    //             "[ZONE_%d IMMINENT] EMPTY",
    //             zone_id);
  }

  prev_imminent_vehicles[zone_id] = imminent_vehicles;
}

//
  std::vector<int> current_red_flag_vehicles; //이번 실행에서 멈춰야하는 차량  id 목록

  if (imminent_vehicles.empty()) {
    // RCLCPP_INFO(node->get_logger(), "[ZONE_%d] Imminent EMPTY", zone_id);
  } else {
    main_cav_id = imminent_vehicles[0]; //아까 imminent_vehicles 거리순으로 정렬해놓음 그래서 젤 가까운게 0번 인덱스

    // 방어 로직: main_cav_id의 경로가 존재하는지 확인
    if (cav_paths.find(main_cav_id) != cav_paths.end() && !cav_paths[main_cav_id].empty()) {
      // RCLCPP_INFO(node->get_logger(), "[ZONE_%d MAIN_CAV] Selected Closest CAV_%d (Dist: %.2fm)",
      //             zone_id, main_cav_id, imminent_with_dist[0].first);

      if (!precollision_vehicles.empty()) {
        std::vector<int> sub_candidates;
        for (int cav_id : precollision_vehicles) {
          if (cav_id == main_cav_id) continue;
          if (cav_paths.find(cav_id) == cav_paths.end() || cav_paths[cav_id].empty()) continue;  // 경로 체크
          if (is_moving_away(cav_id, zone_origin, cav_paths[cav_id])) continue;

          sub_candidates.push_back(cav_id);
        }

        if (!sub_candidates.empty()) {
          cav_sub = select_best_sub_cav(main_cav_id, sub_candidates, zone_origin, overlap_threshold, lookahead_distance);

          if (cav_sub != -1) {
            // RCLCPP_INFO(node->get_logger(), "[ZONE_%d SUB_CAV] Selected Safe CAV_%d", zone_id, cav_sub);
          } else {
            // RCLCPP_ERROR(node->get_logger(), "[ZONE_%d] All candidates overlap! No Sub CAV selected.", zone_id);
          }

          for (int cav_id : sub_candidates) {
          if (cav_id == cav_sub) {
            continue;
          } else {
            current_red_flag_vehicles.push_back(cav_id);
          }
        }
      }
    }
    } // main_cav_id 경로 체크 끝
  }

  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    // RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG STATE CHANGED]", zone_id);

    for (int prev_cav_index : prev_red_flag_vehicles[zone_id]) {
      if (std::find(current_red_flag_vehicles.begin(), current_red_flag_vehicles.end(), prev_cav_index) == current_red_flag_vehicles.end()) {
        // Convert cav_index to actual_cav_id using reverse map
        if (cav_index_map_idx_to_actual.count(prev_cav_index)) {
          int prev_actual_cav_id = cav_index_map_idx_to_actual.at(prev_cav_index);
          auto msg = std_msgs::msg::Int32();
          msg.data = 0;  //0이 그린플래그
          if(red_flag_pubs.count(prev_actual_cav_id)) {
            red_flag_pubs[prev_actual_cav_id]->publish(msg);
            // RCLCPP_INFO(node->get_logger(), "[ZONE_%d GREEN_FLAG] CAV_%d -> RESUME", zone_id, prev_actual_cav_id);
          }
        }
      }
    }

    prev_red_flag_vehicles[zone_id] = current_red_flag_vehicles;
  }

  // 매 루프마다 현재 상태의 RED_FLAG 지속적으로 전송 (상태 유지)
  for (int cav_index : current_red_flag_vehicles) {
    // Convert cav_index to actual_cav_id using reverse map
    if (cav_index_map_idx_to_actual.count(cav_index)) {
      int actual_cav_id = cav_index_map_idx_to_actual.at(cav_index);
      auto msg = std_msgs::msg::Int32();
      msg.data = 1;
      if(red_flag_pubs.count(actual_cav_id)) {
        red_flag_pubs[actual_cav_id]->publish(msg);
      }
    }
  }

  // 상태 변화 시에만 로그
  if (current_red_flag_vehicles != prev_red_flag_vehicles[zone_id]) {
    for (int cav_index : current_red_flag_vehicles) {
      if (cav_index_map_idx_to_actual.count(cav_index)) {
        int actual_cav_id = cav_index_map_idx_to_actual.at(cav_index);
        // RCLCPP_WARN(node->get_logger(), "[ZONE_%d RED_FLAG] CAV_%d (Index=%d) -> STOP (Main=Index_%d, Sub=Index_%d)",
        //             zone_id, actual_cav_id, cav_index, main_cav_id, cav_sub);
      }
    }
  }
}

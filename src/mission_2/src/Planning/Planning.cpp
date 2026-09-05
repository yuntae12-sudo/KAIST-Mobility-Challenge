/*
 * Planning.hpp에 선언된 차선 선택/경로 탐색/겹침 판정 함수들의 구현.
 */
#include "Planning/Planning.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include "Mission/Mission.hpp"
#include "Utils/Utils.hpp"

// 각 차선의 Closest Index를 갱신하고, Overlap 강제 처리 후 다음 차선을 선택/전환한다.
// 반환값은 이번 주기에 선택된 next_lane이다.
int PlanningProcess(int cav_id,
                     const std::vector<bool>& lane_collision,
                     bool zone3_flag, bool zone4_flag, bool zone5_flag,
                     bool zone_collision_flag_in, bool zone2_collision_flag_in,
                     bool stop_flag,
                     std::shared_ptr<rclcpp::Node> node)
{
  // ===== 6. Choose passable lane =====
  int next_lane = choose_lane(lane_collision, current_lane, zone3_flag, zone4_flag, zone5_flag);

  // ===== 7. Lane switch if needed =====
  // Zone1/Zone2/Zone3/Zone4/Zone5 충돌 없고, stop_flag 아니면 차선 변경
  if (!zone_collision_flag_in && !zone2_collision_flag_in && !stop_flag && next_lane != current_lane) {
    change_csv_state(cav_id, next_lane, node);
  }

  return next_lane;
}

// 특정 차선에서 현재 위치와 가장 가까운 인덱스를 찾는다.
int get_lane_start_idx(int lane_id, double cav_x, double cav_y) {
    if (lane_id < 1 || lane_id > 3) return -1;
    if (lane_paths[lane_id].empty()) return -1;

    // Search globally (simple) or logically to keep consistent
    // For simplicity in this logic, we search strictly closest
    int closest_idx = 0;
    double min_dist = 1e10;

    for (size_t i = 0; i < lane_paths[lane_id].size(); ++i) {
        double dist = calculateDistance(cav_x, cav_y,
                                       lane_paths[lane_id][i].x,
                                       lane_paths[lane_id][i].y);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    return closest_idx;
}

// 경로 전체를 탐색해 현재 위치와 가장 가까운 인덱스를 찾는다.
int findClosestPointSimple(const std::vector<PathPoint>& path, double x, double y) {
    if (path.empty()) return -1;
    int best = 0;
    double min_d = 1e10;

    for (size_t i = 0; i < path.size(); i++) {
        double d = calculateDistance(x, y, path[i].x, path[i].y);
        if (d < min_d) {
            min_d = d;
            best = i;
        }
    }
    return best;
}

// 이전 인덱스(ref_idx) 이후 구간만 탐색하여 튐(jitter) 현상을 방지한다.
int findClosestPointAhead(const std::vector<PathPoint>& path,
                         double x, double y, int ref_idx) {
    if (path.empty()) return -1;

    int start_idx = std::max(0, ref_idx);
    int best = start_idx;
    double min_d = 1e10;

    // Search only forward to prevent jitter
    int search_limit = std::min((int)path.size(), start_idx + 200);

    for (int i = start_idx; i < search_limit; i++) {
        double d = calculateDistance(x, y, path[i].x, path[i].y);
        if (d < min_d) {
            min_d = d;
            best = i;
        }
    }
    return best;
}

// 차선별 이전 인덱스를 기억하여, 최초 탐색은 전체 탐색으로, 이후는 전방 탐색으로 처리한다.
int findClosestPoint(const std::vector<PathPoint>& path, double x, double y, int lane) {
    if (lane_start_idx.find(lane) == lane_start_idx.end()) {
        lane_start_idx[lane] = findClosestPointSimple(path, x, y);
        return lane_start_idx[lane];
    }

    int result = findClosestPointAhead(path, x, y, lane_start_idx[lane]);
    lane_start_idx[lane] = result;
    return result;
}

// 현재 위치부터 순환하며 Lookahead 거리 이상 떨어진 목표 Waypoint를 찾는다.
int findWaypoint(const std::vector<PathPoint>& path, double x, double y, double lookahead) {
    int closest_idx = findClosestPointSimple(path, x, y);
    if (closest_idx < 0) return -1;

    int path_size = (int)path.size();

    // 현재 위치부터 경로 길이만큼 순환하며 탐색
    for (int i = 0; i < path_size; i++) {
        int target_idx = (closest_idx + i) % path_size; // 순환 인덱스

        double d = calculateDistance(x, y, path[target_idx].x, path[target_idx].y);
        if (d > lookahead) {
            return target_idx;
        }
    }

    // 못 찾으면 현재 위치 리턴 (혹은 경로의 마지막)
    return closest_idx;
}

// 향후 20개 앞선 지점과 현재 지점의 진행 방향 차이로 커브 여부를 판단한다.
bool isCorner(const std::vector<PathPoint>& path, int closest_idx) {
    int future_offset = 20;
    int path_size = (int)path.size();

    if (closest_idx + future_offset + 1 >= path_size) {
        return false;
    }

    double current_angle = atan2(path[closest_idx + 1].y - path[closest_idx].y,
                                 path[closest_idx + 1].x - path[closest_idx].x);

    int future_idx = closest_idx + future_offset;
    double future_angle = atan2(path[future_idx + 1].y - path[future_idx].y,
                               path[future_idx + 1].x - path[future_idx].x);

    double diff = future_angle - current_angle;
    diff = normalizeAngle(diff);
    diff = fabs(diff);
    if (diff > M_PI / 2) {
        diff = M_PI - diff;
    }

    double threshold_rad = (10.0 * M_PI / 180.0);
    return (diff > threshold_rad);
}

// Lane 2와 Lane 3가 겹치는 구간(인덱스 범위)을 미리 계산한다.
void init_overlap_region() {
    if (lane_paths[2].empty() || lane_paths[3].empty()) {
        std::cout << "[OVERLAP_INIT] Paths are empty. Skipping." << std::endl;
        return;
    }

    double threshold = 0.2; // 겹침 판단 거리 (m)
    int path3_size = (int)lane_paths[3].size();

    int min_idx = -1;
    int max_idx = -1;
    bool in_overlap = false;

    // Lane 3의 모든 점을 순회
    for (int i = 0; i < path3_size; ++i) {
        double p3_x = lane_paths[3][i].x;
        double p3_y = lane_paths[3][i].y;

        // Lane 2에서 가장 가까운 점 찾기 (단순 탐색)
        double min_dist = 1e10;
        for (const auto& p2 : lane_paths[2]) {
            double d = std::hypot(p3_x - p2.x, p3_y - p2.y);
            if (d < min_dist) min_dist = d;
        }

        // 겹침 판정
        if (min_dist <= threshold) {
            if (min_idx == -1) min_idx = i; // 최초 발견 지점
            max_idx = i; // 계속 갱신하여 마지막 지점 저장
            in_overlap = true;
        }
    }

    if (in_overlap && min_idx != -1 && max_idx != -1) {
        overlap_start_idx = min_idx;
        overlap_end_idx = max_idx;
        overlap_detected = true;
        std::cout << "========================================" << std::endl;
        std::cout << "[OVERLAP_INIT] Overlap Region Detected!" << std::endl;
        std::cout << "   Range (Lane 3 Index): " << overlap_start_idx << " ~ " << overlap_end_idx << std::endl;
        std::cout << "========================================" << std::endl;
    } else {
        std::cout << "[OVERLAP_INIT] No significant overlap found." << std::endl;
    }
}

// 현재 Lane 3 위에서의 내 위치(인덱스)가 겹침 구간에 근접했는지 확인한다.
bool check_approaching_overlap(int current_lane3_idx) {
    if (!overlap_detected || overlap_start_idx == -1) return false;

    // 미리 막기 위한 여유 버퍼 (인덱스 개수)
    // 예: 50개 포인트(약 5~10m) 전부터 Lane 3를 막음
    int safety_margin = 30;

    // 순환 경로(Circular) 고려 없이 단순 선형 비교일 경우:
    // [overlap_start - margin]  ~  [overlap_end] 구간에 있으면 true
    int block_start = overlap_start_idx - safety_margin;
    int block_end = overlap_end_idx + safety_margin;

    // 현재 인덱스가 이 구간 안에 들어오면 true
    if (current_lane3_idx >= block_start && current_lane3_idx <= block_end) {
        return true;
    }

    return false;
}

// 충돌 정보와 실선 Zone 여부를 바탕으로 다음 주행 차선을 선택한다.
int choose_lane(const std::vector<bool>& collision_list, int current_lane, bool in_zone3, bool in_zone4, bool in_zone5) {
    std::vector<int> priority;
    priority.push_back(current_lane);
    // NOTE(circular dependency, intentionally kept): is_cav_in_zone3/4() are Mission-owned
    // Zone checks. solid_lane1/solid_lane2 below are computed but never read anywhere in
    // this function (dead calculation), confirmed identical in the pre-refactor main branch
    // code. Kept as-is for exact behavior preservation rather than removed, since this is a
    // structure-only cleanup pass, not an algorithm change.
    bool solid_lane1 = is_cav_in_zone3(cav_x, cav_y);
    bool solid_lane2 = is_cav_in_zone4(cav_x, cav_y);

    if (collision_list[1] && collision_list[2] && collision_list[3]) {
        return current_lane;
    }

    if (in_zone3 && in_zone4 && in_zone5) {
        return current_lane;
    }

    if (current_lane == 1) {
        priority.push_back(2);
    } else if (current_lane == 2) {
        // Standard priority: Check 3 then 1.
        // BUT if collision_list[3] is artificially true (due to overlap),
        // the loop will skip 3 and pick 1.
        priority.push_back(3);
        priority.push_back(1);
    } else if (current_lane == 3) {
        priority.push_back(2);
    }

    for (int lane : priority) {
        if (!collision_list[lane]) {
            return lane;
        }
    }
    return current_lane;
}

// 목표 차선으로 활성 경로를 전환한다. Lane 3 겹침 구간 접근 시 Lane 2로 강제 인식한다.
void change_csv_state(int cav_id, int new_lane, std::shared_ptr<rclcpp::Node> node) {
    if (new_lane < 1 || new_lane > 3) return;

    // 좌표(x,y)를 Lane 3 기준 인덱스로 변환 후 검사
    int current_idx_on_3 = get_lane_start_idx(3, cav_x, cav_y);
    bool is_overlap = check_approaching_overlap(current_idx_on_3);

    if (is_overlap && (new_lane == 3 || new_lane == 2)) {
        RCLCPP_WARN(node->get_logger(),
            "[OVERLAP] Path 2 & 3 merged. Forcing Lane 3 -> Lane 2 recognition.");
        new_lane = 2; // 강제로 2번 차선으로 인식하게 함
    }

    // 이미 해당 차선이면 리턴
    if (new_lane == current_lane) return;

    // 경로 교체 및 상태 업데이트
    integrate_path_vector = lane_paths[new_lane];
    current_lane = new_lane;

    lane_start_idx[new_lane] = get_lane_start_idx(new_lane, cav_x, cav_y);

    RCLCPP_INFO(node->get_logger(),
                "[LANE_SWITCH] Switched to Lane %d (size: %zu, start_idx: %d)",
                new_lane, integrate_path_vector.size(), lane_start_idx[new_lane]);
}

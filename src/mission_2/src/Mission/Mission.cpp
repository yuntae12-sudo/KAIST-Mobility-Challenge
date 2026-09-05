/*
 * Mission.hpp에 선언된 Zone/ROI 충돌 판단, HV 속도 측정, 완주 판정 함수들의 구현.
 */
#include "Mission/Mission.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>

#include "Planning/Planning.hpp"
#include "Utils/Utils.hpp"

// Zone 1~5 충돌 플래그를 갱신하고, 차선별 물리적 충돌 여부를 계산하여 정지 필요 여부를 판단한다.
bool MissionProcess(int cav_id, std::vector<bool>& lane_collision, bool& is_overlap_zone,
                     std::shared_ptr<rclcpp::Node> node)
{
  // ===== 2. ZONE 1 충돌 검사 (복구!) =====
  zone_collision_flag = check_zone_collision();
  zone_cav_flag = is_cav_in_zone(cav_x, cav_y);

  // ===== 2-2. ZONE 2 충돌 검사 =====
  zone2_collision_flag = check_zone2_collision();

  // ===== 2-3. ZONE 3 & 4 & 5 충돌 검사 =====
  zone3_collision_flag = is_cav_in_zone3(cav_x, cav_y);
  zone3_1_collision_flag = is_cav_in_zone3_1(cav_x, cav_y);
  zone3_2_collision_flag = is_cav_in_zone3_2(cav_x, cav_y);
  zone4_collision_flag = is_cav_in_zone4(cav_x, cav_y);
  zone5_collision_flag = is_cav_in_zone5(cav_x, cav_y);

  // ===== 3. Get closest index for each lane =====
  std::vector<int> lane_closest(4);
  for (int lane = 1; lane <= 3; ++lane) {
    lane_closest[lane] = get_lane_start_idx(lane, cav_x, cav_y);
  }

  // 4. Physical Collision Check (동일)
  lane_collision.assign(4, false);
  for (int i = 1; i <= 3; ++i) {
    lane_collision[i] = check_lane_roi_collision(i, lane_closest[i], lane_paths[i]);
  }
  // zone2 충돌 감지 시 1, 2차선 막힘 처리 (안정성 강화)
  if (zone2_collision_flag) {
    lane_collision[2] = true;
    lane_collision[1] = true;
  }
  // zon3_2 주행 시 1차선 막힘 처리(1차선으로 가면 장애물이 많기에 3차선 주행이 rap time 향상에 유리)
  if (current_lane == 2 && zone3_2_collision_flag) {
    lane_collision[1] = true;
    lane_collision[2] = true; // 1,2차선 모두 막음으로 3차선 유지 유도
  }
  // zone3/zone5 는 실선이므로 차선변경 불가능하게 막음 (실선 차선 변경 불가능)
  else if (current_lane == 2 && zone5_collision_flag || zone3_collision_flag) {
    lane_collision[3] = true;
  }
  // zone4 는 실선이므로 차선변경 불가능하게 막음 (실선 차선 변경 불가능)
  else if (current_lane == 3 && zone3_1_collision_flag || zone4_collision_flag) {
    lane_collision[2] = true;
    lane_collision[1] = true;
  }
  else if (zone_cav_flag) {
    lane_collision[3] = true;
    lane_collision[2] = true;
  }

  // =================================================================================
  // [MODIFIED] 미리 계산된 인덱스 기반 Overlap 감지
  // =================================================================================
  // 현재 내 위치가 Lane 3 경로 상에서 어디쯤(인덱스)인지 확인
  int my_idx_on_lane3 = get_lane_start_idx(3, cav_x, cav_y);

  // 겹침 구간(또는 그 직전)에 진입했는지 확인
  is_overlap_zone = check_approaching_overlap(my_idx_on_lane3);

  if (is_overlap_zone) {
    // 1) Lane 3는 막힌 것으로 처리 (미리 차단)
    lane_collision[3] = true;

    // 2) 만약 현재 Lane 3를 달리고 있다면, 강제로 Lane 2로 변경
    if (current_lane == 3) {
      RCLCPP_WARN(node->get_logger(), "[OVERLAP] Approaching merge zone! Forcing Lane 3 -> Lane 2.");
      change_csv_state(cav_id, 2, node);
    }
  }

  // ===== 5. Check if should STOP =====
  bool stop_flag = should_stop(lane_collision, current_lane);
  return stop_flag;
}

// =========================
// Zone 1 Detection Functions (복구!)
// =========================

// HV Zone 안에 HV가 있는지 체크 (각 점 기준 0.1m 반경)
bool is_hv_in_zone(double hv_x, double hv_y) {
    const double detection_radius = 0.1;

    for (const auto& zone_point : hv_zone_polygon) {
        double dist = calculateDistance(hv_x, hv_y, zone_point.x, zone_point.y);
        if (dist <= detection_radius) {
            return true;
        }
    }
    return false;
}

// CAV Zone 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone_start.x;
    double y1 = cav_zone_start.y;
    double x2 = cav_zone_end.x;
    double y2 = cav_zone_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE1_DETECTION] CAV in CAV Zone1!" << std::endl;
        return true;
    }

    return false;
}

// Zone 1 충돌 검사: HV Zone에 HV 있고 + CAV Zone에 CAV 있으면 충돌
bool check_zone_collision() {
    bool cav_in_zone1 = is_cav_in_zone(cav_x, cav_y);
    if (!cav_in_zone1) {
        return false;
    }

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        if (is_hv_in_zone(hv_pos.first, hv_pos.second)) {
            std::cout << "[ZONE_COLLISION] HV_" << hv_id << " in HV Zone + CAV in CAV Zone!" << std::endl;
            return true;
        }
    }
    return false;
}

// =========================
// Zone 2 Detection Functions (추가!)
// =========================

// HV Zone 2 안에 HV가 있는지 체크 (선분으로부터 거리)
bool is_hv_in_zone2(double hv_x, double hv_y) {
    const double detection_radius = 0.1;

    double x1 = hv_zone2_start.x;
    double y1 = hv_zone2_start.y;
    double x2 = hv_zone2_end.x;
    double y2 = hv_zone2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(hv_x, hv_y, x1, y1) <= detection_radius;
    }

    double t = ((hv_x - x1) * (x2 - x1) + (hv_y - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(hv_x, hv_y, closest_x, closest_y);

    return dist <= detection_radius;
}

// CAV Zone 2 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone2(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone2_start.x;
    double y1 = cav_zone2_start.y;
    double x2 = cav_zone2_end.x;
    double y2 = cav_zone2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    return dist <= detection_radius;
}

// Zone 2 충돌 검사: HV Zone 2에 HV 있고 + CAV Zone 2에 CAV 있으면 충돌
bool check_zone2_collision() {
    bool cav_in_zone2 = is_cav_in_zone2(cav_x, cav_y);

    if (!cav_in_zone2) {
        return false;
    }

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        if (is_hv_in_zone2(hv_pos.first, hv_pos.second)) {
            std::cout << "[ZONE2_COLLISION] HV_" << hv_id << " in HV Zone2 + CAV in CAV Zone2!" << std::endl;
            return true;
        }
    }

    return false;
}

// =========================
// Zone 3 & 4 & 5 Detection Functions (실선 구간 차선 변경 불가 로직 추가하기 위함)
// =========================

// CAV Zone 3 안에 CAV가 있는지 체크 (선분으로부터 거리)
bool is_cav_in_zone3(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone3_start.x;
    double y1 = cav_zone3_start.y;
    double x2 = cav_zone3_end.x;
    double y2 = cav_zone3_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE3_DETECTION] CAV in CAV Zone3!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone3_1(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone3_1_start.x;
    double y1 = cav_zone3_1_start.y;
    double x2 = cav_zone3_1_end.x;
    double y2 = cav_zone3_1_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE3_1_DETECTION] CAV in CAV Zone3_1!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone3_2(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone3_2_start.x;
    double y1 = cav_zone3_2_start.y;
    double x2 = cav_zone3_2_end.x;
    double y2 = cav_zone3_2_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE3_2_DETECTION] CAV in CAV Zone3_2!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone4(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone4_start.x;
    double y1 = cav_zone4_start.y;
    double x2 = cav_zone4_end.x;
    double y2 = cav_zone4_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE4_DETECTION] CAV in CAV Zone4!" << std::endl;
        return true;
    }

    return false;
}

bool is_cav_in_zone5(double cav_x_in, double cav_y_in) {
    const double detection_radius = 0.1;

    double x1 = cav_zone5_start.x;
    double y1 = cav_zone5_start.y;
    double x2 = cav_zone5_end.x;
    double y2 = cav_zone5_end.y;

    double line_len_sq = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

    if (line_len_sq < 1e-6) {
        return calculateDistance(cav_x_in, cav_y_in, x1, y1) <= detection_radius;
    }

    double t = ((cav_x_in - x1) * (x2 - x1) + (cav_y_in - y1) * (y2 - y1)) / line_len_sq;
    t = std::max(0.0, std::min(1.0, t));

    double closest_x = x1 + t * (x2 - x1);
    double closest_y = y1 + t * (y2 - y1);

    double dist = calculateDistance(cav_x_in, cav_y_in, closest_x, closest_y);

    if (dist <= detection_radius) {
        std::cout << "[ZONE5_DETECTION] CAV in CAV Zone5!" << std::endl;
        return true;
    }

    return false;
}

// 특정 경로 인덱스 지점에 HV가 threshold 이내로 접근했는지 확인한다.
bool is_collision(int idx, const std::vector<PathPoint>& path,
                 double threshold) {
    if (idx < 0 || idx >= (int)path.size()) return false;

    const PathPoint& point = path[idx];

    for (const auto& [hv_id, hv_pos] : hv_positions) {
        double dist = calculateDistance(point.x, point.y, hv_pos.first, hv_pos.second);
        if (dist <= threshold) {
            return true;
        }
    }
    return false;
}

// 차선의 현재 위치 주변 여러 오프셋 지점에 대해 HV 충돌 여부를 검사한다 (순환 인덱스 처리 포함).
bool check_lane_roi_collision(int lane_id, int start_idx,
                             const std::vector<PathPoint>& path) {
    if (lane_id < 1 || lane_id > 3) return false;
    if (path.empty()) return false; // 예외 처리 추가

    int path_size = (int)path.size(); // 전체 경로 크기

    // Check points ahead
    std::vector<int> roi_offsets = {-20, -15, -10, -5, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 70, 80, 90};

    for (int offset : roi_offsets) {
        int check_idx = start_idx + offset;

        // [수정] 순환(Circular) 인덱스 처리
        // 범위를 벗어나면 반대편으로 넘김
        if (check_idx >= path_size) {
            check_idx -= path_size;
        } else if (check_idx < 0) {
            check_idx += path_size;
        }

        if (is_collision(check_idx, path)) {
            return true;
        }
    }
    return false;
}

// 현재 차선에서 정지가 필요한지 판단한다 (규칙은 차선별로 다르다).
bool should_stop(const std::vector<bool>& collision_list, int current_lane) {
    // Lane 1: Lane 2가 막히면 정지
    if (current_lane == 1) {
        return collision_list[1] && collision_list[2];
    }
    // Lane 3: Lane 2가 막히면 정지
    else if (current_lane == 3) {
        return collision_list[3] && collision_list[2];
    }
    // Lane 2: Lane 1과 Lane 3이 *모두* 막혀야 정지 (하나라도 뚫려있으면 회피 가능)
    else if (current_lane == 2) {
        return collision_list[2] && collision_list[1] && collision_list[3];
    }

    return false;
}

// =========================
// HV 속도 측정 콜백 (Moving Average 방식)
// =========================
void MeasureHVVelocity(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                       int hv_id,
                       std::shared_ptr<rclcpp::Node> node) {

    HVState& state = hv_states[hv_id];
    state.id = hv_id;

    rclcpp::Time current_time = msg->header.stamp;
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;

    // 1. 첫 메시지 처리
    if (state.is_first_msg) {
        state.prev_x = current_x;
        state.prev_y = current_y;
        state.last_time = current_time;
        state.is_first_msg = false;
        return;
    }

    // 2. 시간 차이 계산
    double dt = (current_time - state.last_time).seconds();

    // dt가 너무 작으면 계산 생략 (0.001초 미만 등)
    if (dt < 0.001) return;

    // 3. 이동 거리 및 속도 계산
    double dx = current_x - state.prev_x;
    double dy = current_y - state.prev_y;
    double dist = std::hypot(dx, dy);

    double current_vel = dist / dt;

    // 4. 이동 평균 필터 적용
    if (state.vel_buffer.size() >= state.buffer_size) {
        state.vel_buffer.pop_front();
    }
    state.vel_buffer.push_back(current_vel);

    double sum = std::accumulate(state.vel_buffer.begin(), state.vel_buffer.end(), 0.0);
    double avg_vel = sum / state.vel_buffer.size();

    // 5. 노이즈 및 상한선 처리
    if (avg_vel < 0.01) avg_vel = 0.0; // 정지 상태 처리
    if (avg_vel > 2.5) avg_vel = 2.5;  // 비정상적으로 튀는 값 제한

    // 6. 전역 변수 업데이트
    if (hv_id == 20) {
        measured_hv20_vel = avg_vel;
    } else if (hv_id == 24) {
        measured_hv24_vel = avg_vel;
    }

    // 7. 상태 업데이트
    state.prev_x = current_x;
    state.prev_y = current_y;
    state.last_time = current_time;
}

// 출발선 통과 여부로 Lap을 갱신하고, 5랩 완료 시 완주 처리한다.
bool is_race_over(CavState& cav) {
    // 1. 초기화 안 된 경우 시작점 설정
    if (!cav.is_initialized) {
        cav.id = 1; // 기본 ID 설정 (필요시 수정)
        cav.start_x = cav_x;
        cav.start_y = cav_y;
        cav.start_yaw = cav_yaw;
        std::cout << "[CAV " << cav.id << "] Start Point Set: (" << cav.start_x << ", " << cav.start_y << ")" << std::endl << std::endl;
        cav.is_in_line = true; // 시작하자마자는 라인 안에 있다고 가정
        cav.is_initialized = true;
    }

    double start_dx = cav_x - cav.start_x;
    double start_dy = cav_y - cav.start_y;

    double start_cos = std::cos(cav.start_yaw);
    double start_sin = std::sin(cav.start_yaw);

    // 로컬 좌표로 변환 (start_x, start_y 기준)
    double local_lon = start_cos * start_dx + start_sin * start_dy;
    double local_lat = -start_sin * start_dx + start_cos * start_dy;

    // 결승선 통과 조건 (출발점 기준 앞 0~0.5m, 좌우 0.8m 이내)
    bool line_thickness = (local_lon >= 0.0 && local_lon <= 0.5);
    bool line_width = (std::abs(local_lat) <= 0.8);
    bool on_finish_line = line_thickness && line_width;

    if (on_finish_line) {
        // 이전에 라인 밖에 있었다가 들어온 경우 -> 랩 카운트 증가
        if (!cav.is_in_line) {
            cav.current_lap++;
            cav.is_in_line = true;

            std::cout << "\n>>> [LAP UPDATE] Passed Gate! Lap " << cav.current_lap << " / 5 <<<" << std::endl;
            std::cout << "    (Forward Dist: " << local_lon << "m, Side: " << local_lat << "m)" << std::endl;

            if (cav.current_lap >= 5) {
                cav.is_finished = true;
                std::cout << ">>> [FINISH] 5 LAPS COMPLETED! STOPPING CAR... <<<" << std::endl;
            }
        }
    } else {
        // 라인 밖으로 벗어남 (한 바퀴 도는 중)
        cav.is_in_line = false;
    }

    return cav.is_finished;
}

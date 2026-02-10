#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp> 

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <memory>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

// =========================
// Path data
// =========================
struct integrate_path_struct {
  double x;
  double y;
};

static std::vector<integrate_path_struct> integrate_path_vector;

// =========================
// Localization globals
// =========================
inline double x_m = 0.0;
inline double y_m = 0.0;
inline double z_m = 0.0;

inline double x_q = 0.0;
inline double y_q = 0.0;
inline double z_q = 0.0;
inline double w_q = 0.0;

inline void get_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
                     double &x_m, double &y_m, double &z_m,
                     double &x_q, double &y_q, double &z_q, double &w_q)
{
  x_m = msg->pose.position.x;
  y_m = msg->pose.position.y;
  z_m = msg->pose.position.z;

  x_q = msg->pose.orientation.x;
  y_q = msg->pose.orientation.y;
  z_q = msg->pose.orientation.z;
  w_q = msg->pose.orientation.w;
}

// =========================
// Controller state
// =========================
struct ControllerState
{
  double speed_mps{0.5};
  double lookahead_m{0.3};
  double max_yaw_rate{2.0}; // ** 5.5 (noise error) -> 2.5 (tuning) ** -> 2.0

  double prev_x{0.0};
  double prev_y{0.0};
  double prev_yaw{0.0};
  bool has_prev{false};

  int pos_count{0};
  int red_flag{0}; 
  int yellow_flag{0};  // Yellow flag for speed control
  bool tower_mode{false};
  double target_linear_x{0.0};
  
  // Velocity ramp control
  double current_velocity{0.0};  // 실제 발행되는 속도
//   double max_acceleration{0.8};  // 최대 가속도 (m/s²) - 조정 가능
  double max_acceleration{2.5};  // 최대 가속도 (m/s²) - 조정 가능
  
  // ✅ [추가] Red flag 상태 추적 (정지 후 느리게 속도 올림)
  int red_flag_hold_count{0};  // Red flag 지속 카운트
  const int RED_FLAG_RELEASE_STABILIZE_CYCLES = 100;  // ~1초 (50Hz에서)
  double max_acceleration_after_red_flag{2.5};  // Red flag 해제 후 느린 가속도
  
  // ✅ [추가] 시작 3초 정지
  rclcpp::Time start_stop_time;  // 시작 정지 시작 시간
  bool is_start_stop_active{false};  // 정지 중인지 여부
  const int START_STOP_DURATION_MS = 4000;  // 3초
  
  rclcpp::Time last_update_time;  // 마지막 업데이트 시간
};

// =========================
// CAV Structures
// =========================
struct CavState {
    int id;                 
    double start_x;         
    double start_y;         
    int current_lap;        
    bool is_initialized;    
    bool is_in_zone;        
    bool finished;
    rclcpp::Time lap_start_time;  // Lap start time for lap timing
};

// =========================
// Helper Functions
// =========================
static double yawFromQuat(const geometry_msgs::msg::Quaternion& qmsg, double fallback_yaw)
{
  const double norm2 = qmsg.x*qmsg.x + qmsg.y*qmsg.y + qmsg.z*qmsg.z + qmsg.w*qmsg.w;
  if (norm2 <= 1e-6) return fallback_yaw;

  tf2::Quaternion q(qmsg.x, qmsg.y, qmsg.z, qmsg.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

static int readCavIdFromEnvOrDefault(int default_id)
{
  const char* s = std::getenv("CAV_ID");
  if (!s || std::string(s).empty()) return default_id;
  try { return std::stoi(s); } catch (...) { return default_id; }
}

static std::string twoDigitId(int id)
{
  if (id < 0) id = 0;
  if (id > 99) id = 99;

  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << id;
  return oss.str();
}

static bool loadPathCsv(const std::string& csv_path, std::vector<integrate_path_struct>& out)
{
  std::ifstream file(csv_path);
  if (!file.is_open()) return false;

  out.clear();
  std::string line;
  if (std::getline(file, line)) {
    std::stringstream ss(line);
    double x, y; char comma;
    if (ss >> x >> comma >> y) {
      out.push_back({x, y});
    }
  }

  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    double x, y; char comma;
    if (!(ss >> x)) continue;
    if (!(ss >> comma)) continue;
    if (!(ss >> y)) continue;
    if (std::isnan(x) || std::isnan(y)) continue;
    out.push_back({x, y});
  }

  return out.size() >= 2;
}

// =========================
// Waypoint search
// =========================
inline int closest_index = 0;

void GetLd(ControllerState& st) {
//   double gain_ld = 0.6; // 0.4 -> 0.6 ** tuning **
  // double gain_ld = 0.7; // 0.4 -> 0.6 ** tuning ** 0.6 -> 0.7 
//   double max_ld  = 0.355;
  double max_ld  = 0.4;  // 
  double min_ld  = 0.3; // 0.15 -> 0.1 -> 0.2 // ** 0.33 -> 0.23 
  // double velocity = st.speed_mps; 
//   double ld = gain_ld * velocity;
  double ld = 3.0;
  st.lookahead_m = max(min_ld, std::min(max_ld, ld));
}

static int findClosestPoint(const std::vector<integrate_path_struct>& path, double x_m, double y_m)
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

// int findWaypoint (const vector<integrate_path_struct>& integrate_path_vector, double x_m, double y_m, const double L_d) {
//     if (integrate_path_vector.empty()) { cout <<"Path Empty" << endl; return -1; }
//     int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
//     for (int i = closest_idx; i < (int)integrate_path_vector.size(); ++i) {
//         double d = hypot(integrate_path_vector[i].x - x_m, integrate_path_vector[i].y - y_m);
//         if (d > L_d) { return i; }
//     }
//     return integrate_path_vector.size() - 1;
// }
int findWaypoint (const vector<integrate_path_struct>& integrate_path_vector, double x_m, double y_m, const double L_d) {
    if (integrate_path_vector.empty()) { cout <<"Path Empty" << endl; return -1; }
    
    int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
    int path_size = (int)integrate_path_vector.size();

    // Loop를 통해 순환 탐색 (최대 전체 경로 길이만큼 확인)
    for (int i = 0; i < path_size; ++i) {
        // [핵심 변경] 모듈러 연산(%)을 사용하여 인덱스가 끝을 넘어가면 0으로 돌아오게 함
        int target_path_idx_with_LD = (closest_idx + i) % path_size;

        double d = hypot(integrate_path_vector[target_path_idx_with_LD].x - x_m, integrate_path_vector[target_path_idx_with_LD].y - y_m);
        if (d > L_d) { 
            return target_path_idx_with_LD; 
        }
    }
    
    // 혹시라도 못 찾으면 현재 위치 바로 다음 점 반환 (순환 고려)
    return (closest_idx + 1) % path_size;
}

double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool isCorner(const vector<integrate_path_struct>& integrate_path_vector, double /*L_d*/, int closest_idx) {
    // int future_offset = 20; // 20 -> 60
    int future_offset = 60;
    int path_size = (int)integrate_path_vector.size();
    if (closest_idx + future_offset + 1 >= path_size) { return false; }

    double cur_ang = atan2(integrate_path_vector[closest_idx + 1].y - integrate_path_vector[closest_idx].y, 
                           integrate_path_vector[closest_idx + 1].x - integrate_path_vector[closest_idx].x);
    int idx_future_start = closest_idx + future_offset;
    double fut_ang = atan2(integrate_path_vector[idx_future_start + 1].y - integrate_path_vector[idx_future_start].y, 
                           integrate_path_vector[idx_future_start + 1].x - integrate_path_vector[idx_future_start].x);
    
    double diff = fabs(normalizeAngle(fut_ang - cur_ang));
    if (diff > M_PI / 2) { diff = M_PI - diff; }
    
    double threshold_deg = 10.0; 
    double threshold_rad = threshold_deg * (M_PI / 180.0);
    return (diff > threshold_rad);
}
// *** 
void planVelocity(ControllerState& st, bool isCorner) {
    // 1.7 ==> 1:05
    // 1.8 ==> 1:04
    // 2.0 ==> 1:00
    if (!isCorner) {st.speed_mps = 1.8; } else { st.speed_mps = 1.5; }     // < ---------------1------------------
    // if (!isCorner) {st.speed_mps = 1.7; } else { st.speed_mps = 1.5; }  //  < --------------2 [v]-------------------
    // if (!isCorner) {st.speed_mps = 2.0; } else { st.speed_mps = 1.5; }  //  < --------------3-------------------
    // if (!isCorner) {st.speed_mps = 2.0; } else { st.speed_mps = 1.5; }  //  < --------------4-------------------
    // if (!isCorner) {st.speed_mps = 1.9; } else { st.speed_mps = 1.5; }  //  < --------------5-------------------
    // if (!isCorner) {st.speed_mps = 1.9; } else { st.speed_mps = 1.5; }  //  < --------------6--------------------
    // if (!isCorner) {st.speed_mps = 1.8; } else { st.speed_mps = 1.5; }  //  < --------------7------------------
    // if (!isCorner) {st.speed_mps = 1.8; } else { st.speed_mps = 1.5; }  //  < --------------8-------------------


}

// *** 속도 래프 함수: 최대 가속도 제한 ***
double applyVelocityRamp(ControllerState& st, double target_velocity, rclcpp::Time current_time) {
    // 첫 호출 시 초기화
    if (st.last_update_time.nanoseconds() == 0) {
        st.last_update_time = current_time;
        st.current_velocity = 0.0;
        return 0.0;
    }
    
    // 경과 시간 계산
    double dt = (current_time - st.last_update_time).seconds();
    if (dt < 0.001) dt = 0.001;  // 최소 1ms
    if (dt > 0.1) dt = 0.2;      // 최대 100ms (센서 끊김 방지)
    
    st.last_update_time = current_time;
    
    // 목표 속도와 현재 속도의 차이
    double velocity_diff = target_velocity - st.current_velocity;
    
    // ✅ [수정] Red flag 해제 후 일시적으로 낮은 가속도 사용
    double effective_max_accel = st.max_acceleration;
    
    if (st.red_flag == 0 && st.red_flag_hold_count < st.RED_FLAG_RELEASE_STABILIZE_CYCLES) {
        // Red flag가 해제되었지만, 안정화 기간 중
        st.red_flag_hold_count++;
        effective_max_accel = st.max_acceleration_after_red_flag;  // 느린 가속도 적용
    } else if (st.red_flag == 1) {
        // Red flag 중: 카운트 리셋
        st.red_flag_hold_count = 0;
    }
    
    // 최대 가속도에 따른 속도 변화량 계산
    double max_delta_v = effective_max_accel * dt;
    
    // 속도 제한 (급격한 변화 방지)
    if (velocity_diff > max_delta_v) {
        st.current_velocity += max_delta_v;
    } else if (velocity_diff < -max_delta_v) {
        st.current_velocity -= max_delta_v;
    } else {
        st.current_velocity = target_velocity;
    }
    
    // 속도가 음수가 되지 않도록 제한
    st.current_velocity = std::max(0.0, st.current_velocity);
    
    return st.current_velocity;
}

bool CheckAllFinished(const std::vector<CavState>& cav_list, int /*vehicle_count*/) {
    int finished_count = 0;
    // for (int i = 1; i <= vehicle_count; ++i) {
    //     if (cav_list[i].finished) finished_count++;
    // }
    // if (finished_count == vehicle_count) return true;
    // return false;
    for (int i = 1; i <= 4; ++i) {
        if (cav_list[i].finished) finished_count++;
    }
    if (finished_count == 4) return true;
    return false;
}

// Global mission completion flag
static bool mission_completed = false;
static int stop_publish_count = 0;

void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int cav_id, std::vector<CavState>& states, int my_cav_index, std::shared_ptr<ControllerState> st) {
    CavState& current_cav = states[cav_id];
    double current_x = msg->pose.position.x;
    double current_y = msg->pose.position.y;
    
    bool is_my_cav = (cav_id == my_cav_index);  // 자신의 CAV인지 확인

    if (!current_cav.is_initialized) {
        current_cav.start_x = current_x;
        current_cav.start_y = current_y;
        current_cav.is_initialized = true;
        current_cav.is_in_zone = false;  // ← FALSE로 시작! 첫 프레임에서 영역 밖으로 설정
        current_cav.lap_start_time = msg->header.stamp;
        
        // ✅ [추가] 시작 3초 정지 활성화
        st->is_start_stop_active = true;
        st->start_stop_time = msg->header.stamp;
        if (is_my_cav) {
            std::cout << "[CAV_index " << cav_id << "] 3-second initial stop activated" << std::endl;
        }
        
        // [Zone Info Commented Out]
        // if (is_my_cav) {
        //     std::cout << "[DEBUG] CAV_index " << cav_id << " initialized at (" << current_x << ", " << current_y << ")" << std::endl;
        // }
        return;
    }

    if (current_cav.finished) return;

    double dist_to_start = std::hypot(current_x - current_cav.start_x, current_y - current_cav.start_y);

    if (dist_to_start < 0.3) { // 0.15 -> 0.3
        if (!current_cav.is_in_zone) {
            current_cav.current_lap += 1;
            current_cav.is_in_zone = true;
            
            // Calculate lap time
            double lap_time_sec = (rclcpp::Time(msg->header.stamp) - current_cav.lap_start_time).seconds();
            current_cav.lap_start_time = rclcpp::Time(msg->header.stamp);  // Reset for next lap
            
            if (is_my_cav) {
                std::cout << "═══════════════════════════════════════════════════════════════" << std::endl;
                std::cout << "🏁 [CAV_index " << cav_id << "] Lap: " << current_cav.current_lap << "/5 (";
                std::cout << (current_cav.current_lap * 100 / 5) << "% Complete)" << std::endl;
                std::cout << "   Lap Time: " << std::fixed << std::setprecision(2) << lap_time_sec << " sec" << std::endl;
                // [Zone Info Commented Out]
                std::cout << "   Current Pos: (" << std::fixed << std::setprecision(2) << current_x << ", " << current_y << ")" << std::endl;
                std::cout << "═══════════════════════════════════════════════════════════════" << std::endl << std::endl;
            }
            
            if (current_cav.current_lap >= 6) { // ** LAP COUNT 5 -> 6 **
                current_cav.finished = true;
                if (is_my_cav) {
                    std::cout << "🎉 [CAV_index " << cav_id << "] ✓ FINISHED 5 LAPS! ✓" << std::endl << std::endl;
                }
            }
        }
    } else {
        if (current_cav.is_in_zone) {
            current_cav.is_in_zone = false;
            // if (is_my_cav) {
            //     std::cout << "[DEBUG] CAV " << cav_id << " left zone (dist=" << std::fixed << std::setprecision(2) << dist_to_start << "m)" << std::endl;
            // }
        }
    }
}

// =========================
// main
// =========================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("control");
  auto st   = std::make_shared<ControllerState>();

  int total_vehicle_count = 32;
  std::vector<CavState> cav_list(total_vehicle_count + 1); 
  for(int i=1; i<(int)cav_list.size(); ++i) cav_list[i] = {i, 0.0, 0.0, 0, false, false, false, rclcpp::Time()};
  
  // 실제 참여 차량 수
  int actual_vehicle_count = 0;

  int cav_id_default = readCavIdFromEnvOrDefault(1);
  node->declare_parameter<int>("cav_id", cav_id_default);
  const int actual_cav_id = node->get_parameter("cav_id").as_int();

  if(actual_cav_id >= (int)cav_list.size()) {
      RCLCPP_ERROR(node->get_logger(), "CAV_ID(%d) Error", actual_cav_id);
      return -1;
  }

  // Get CAV_IDS from environment variable to determine cav_index (1-4)
  const char* cav_ids_env = std::getenv("CAV_IDS");
  int cav_index = 1;  // Default to 1
  if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
      std::string cav_ids_str(cav_ids_env);
      std::stringstream ss(cav_ids_str);
      std::string token;
      int idx = 1;
      while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
              actual_vehicle_count++;  // 실제 참여 차량 수 증가
              int id = std::stoi(token);
              if (id == actual_cav_id) {
                  cav_index = idx;
                  break;
              }
              idx++;
          }
      }
  }
// sudo chmod 666 /dev/ttyUSB0   # 임시(재부팅/재연결 시 원복될 수 있음)
// ./cav_4_SDK.sh

  const std::string my_id_str = twoDigitId(actual_cav_id);  // Use actual_cav_id for all topics
  const std::string accel_topic = "/CAV_" + my_id_str + "_accel"; 
  const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";
  const std::string yellow_flag_topic = "/CAV_" + my_id_str + "_YELLOW_FLAG";
  const std::string target_vel_topic = "/CAV_" + my_id_str + "_target_vel";
  const std::string cmd_vel_topic = "/CAV_" + my_id_str + "/cmd_vel_"; // motor_topic

//   RCLCPP_INFO(node->get_logger(), "My Actual_CAV_ID=%d, Mapped_Index=%d, Actual_Vehicle_Count=%d", actual_cav_id, cav_index, actual_vehicle_count);

  node->declare_parameter<double>("speed_mps", 0.5);
  node->declare_parameter<double>("lookahead_m", 0.4);
  node->declare_parameter<double>("max_yaw_rate", 5.5); // [핵심] 고속 주행을 위해 Yaw Rate 제한 대폭 해제 // ** 5.5 -> 2.5 ?** // ****
  node->declare_parameter<std::string>("path_csv", "/home/aim/TEAM_AIM/src/global_path/path.csv");

  st->speed_mps    = node->get_parameter("speed_mps").as_double();
  st->lookahead_m  = node->get_parameter("lookahead_m").as_double();
  st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

  // Get TEAM_AIM_HOME environment variable (default: /home/aim/TEAM_AIM)
  const char* team_aim_home = std::getenv("TEAM_AIM_HOME");
  std::string base_path = (team_aim_home != nullptr && strlen(team_aim_home) > 0) 
                          ? std::string(team_aim_home) 
                          : std::string("/home/aim/TEAM_AIM");
  const std::string path_with_id_csv = base_path + "/src/global_path/" + "path_mission3_" + std::string(2 - std::to_string(cav_index).length(), '0') + std::to_string(cav_index) + ".csv";
  if (!loadPathCsv(path_with_id_csv, integrate_path_vector)) {
    RCLCPP_FATAL(node->get_logger(), "Failed to load path csv: %s", path_with_id_csv.c_str());
    rclcpp::shutdown(); return 1;
  }
//   RCLCPP_INFO(node->get_logger(), "Loaded path: %zu waypoints", integrate_path_vector.size());
 
  auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());
  auto cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::QoS(10));

  auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      red_flag_topic, 1, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->red_flag = msg->data; });

  auto yellow_flag_sub = node->create_subscription<std_msgs::msg::Int32>(
      yellow_flag_topic, 1, [node, st](const std_msgs::msg::Int32::SharedPtr msg) { st->yellow_flag = msg->data; });

  auto vel_sub = node->create_subscription<std_msgs::msg::Float64>(
      target_vel_topic, 1, 
      [st](const std_msgs::msg::Float64::SharedPtr msg) {
          if (msg->data < 0.0) {
              st->tower_mode = false;
          } else {
              st->tower_mode = true;
              st->target_linear_x = msg->data;
          }
      }
  );

  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_subs;

  // Get actual CAV IDs from environment variable (reuse existing cav_ids_env)
  // PARSING FIXED
  std::vector<int> all_active_cav_ids;
  if (cav_ids_env != nullptr && strlen(cav_ids_env) > 0) {
      std::string cav_ids_str(cav_ids_env);
      std::stringstream ss(cav_ids_str);
      std::string token;
      while (std::getline(ss, token, ',')) {
          token.erase(0, token.find_first_not_of(" \t"));
          token.erase(token.find_last_not_of(" \t") + 1);
          if (!token.empty()) {
              all_active_cav_ids.push_back(std::stoi(token));
          }
      }
  }

  // Subscribe to ALL CAV topics while maintaining CSV index mapping
  for (size_t i = 0; i < all_active_cav_ids.size(); ++i) {
      int csv_index = (int)i + 1;  // 1, 2, 3, 4 (for CSV path mapping)
      int actual_subscribe_cav_id = all_active_cav_ids[i];  // actual CAV ID to subscribe
      
      std::string target_topic = "/CAV_" + twoDigitId(actual_subscribe_cav_id);
      
      auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
          target_topic, rclcpp::SensorDataQoS().keep_last(15),
          [node, st, accel_pub, cmd_vel_pub, &cav_list, actual_cav_id, csv_index, cav_index, actual_vehicle_count, cmd_vel_topic](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
          {
            // Update lap tracking for this vehicle
            PoseCallback(msg, csv_index, cav_list, cav_index, st);

            // Print lap status of all vehicles periodically
            // static int lap_status_counter = 0;
            // if (lap_status_counter++ % 500 == 0) {  // Print every 500 cycles (~10 seconds at 50Hz)
            //     std::cout << "\n=== LAP STATUS ===";
            //     for (int i = 1; i <= actual_vehicle_count; ++i) {
            //         std::cout << " | CAV_" << i << ": " << cav_list[i].current_lap << "/5";
            //     }
            //     std::cout << " ===\n" << std::endl;
            // }

            // Check mission completion and set global flag
            if (CheckAllFinished(cav_list, actual_vehicle_count) && !mission_completed) {
                mission_completed = true;
                // std::cout << ">>> [ALL " << actual_vehicle_count << " VEHICLES FINISHED] Starting continuous STOP publish! <<<" << std::endl;
            }

            // Execute control ONLY for my own CAV
            if (csv_index == cav_index) {
                // 미션 완료 후 계속 정지 명령 발행 (실제 로봇이 멈추도록)
                if (mission_completed) {
                    geometry_msgs::msg::Accel stop_cmd;
                    stop_cmd.linear.x = 0.0; stop_cmd.linear.y = 0.0; stop_cmd.linear.z = 0.0;
                    stop_cmd.angular.x = 0.0; stop_cmd.angular.y = 0.0; stop_cmd.angular.z = 0.0;
                    accel_pub->publish(stop_cmd);
                    
                    geometry_msgs::msg::Twist stop_twist;
                    // fix : linear.x 0 -> -0.005
                    // ** 
                    stop_twist.linear.x = -0.005; stop_twist.linear.y = 0.0; stop_twist.linear.z = 0.0;
                    stop_twist.angular.x = 0.0; stop_twist.angular.y = 0.0; stop_twist.angular.z = 0.0;
                    cmd_vel_pub->publish(stop_twist);
                    
                    // if (stop_publish_count++ % 100 == 0) {
                    //     std::cout << "[STOP] Continuous STOP command published to " << cmd_vel_topic << " (" << stop_publish_count << ")" << std::endl;
                    // }
                    return;
                }

                get_pose(msg, x_m, y_m, z_m, x_q, y_q, z_q, w_q);
                double yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);
                if (st->has_prev && std::hypot(x_m - st->prev_x, y_m - st->prev_y) > 1e-4) {
                    yaw = std::atan2(y_m - st->prev_y, x_m - st->prev_x);
                }
                st->prev_x = x_m; st->prev_y = y_m; st->prev_yaw = yaw; st->has_prev = true;
                
                // 퓨어퍼슈트 로직
                // 1. Closest Point & Velocity
                int closest_idx = findClosestPoint(integrate_path_vector, x_m, y_m);
                bool corner_detected = isCorner(integrate_path_vector, st->lookahead_m, closest_idx);

                double current_target_speed;
                if (st->tower_mode) {
                    current_target_speed = st->target_linear_x;
                } else {
                    planVelocity(*st, corner_detected);
                    current_target_speed = st->speed_mps;
                }
                
                // *** [핵심] 속도 래프 적용 - 급가속 방지 및 헤딩 안정화 ***
                double ramped_speed = applyVelocityRamp(*st, current_target_speed, msg->header.stamp);
                
                st->speed_mps = current_target_speed;  // 목표값 유지
                GetLd(*st); 

                // 2. Pure Pursuit Geometry
                int target_path_idx = findWaypoint(integrate_path_vector, x_m, y_m, st->lookahead_m);

                const double dx = integrate_path_vector[target_path_idx].x - x_m;
                const double dy = integrate_path_vector[target_path_idx].y - y_m;

                const double cy = std::cos(yaw);
                const double sy = std::sin(yaw);
                const double x_v =  cy * dx + sy * dy;
                const double y_v = -sy * dx + cy * dy;
                const double pp_dist = std::max(1e-3, std::hypot(x_v, y_v));
                const double kappa = (2.0 * y_v) / (pp_dist * pp_dist);
                
                // *** [수정] ramped_speed를 사용하여 각속도도 부드럽게 제어 ***
                double wz = ramped_speed * kappa;
                wz = std::clamp(wz, -st->max_yaw_rate, st->max_yaw_rate);

                /*
                kappa - 곡률
                ws(각속도) - 속도 * 곡률 = 회전각도 
                clamp를 통해, 최대 조향을 제한
                마지막으로 twist.cmd.angular_z = ws 로 각속도를 전달
                */

                // [New] Print current speed info periodically
                // static int speed_print_counter = 0;
                // if (speed_print_counter++ % 50 == 0) {  // Print every 50 cycles (~1 second at 50Hz)
                //     std::cout << "[CAV " << actual_subscribe_cav_id << "] Speed: " << std::fixed << std::setprecision(2) << current_target_speed << " m/s" << std::endl;
                // }


                // Command Publish
                geometry_msgs::msg::Accel cmd;
                geometry_msgs::msg::Twist twist_cmd;
                
                // ✅ [추가] 시작 3초 정지 체크
                if (st->is_start_stop_active) {
                    double elapsed_ms = (rclcpp::Time(msg->header.stamp) - st->start_stop_time).seconds() * 1000.0;
                    if (elapsed_ms < 3000) {
                        // 3초 동안 정지
                        cmd.linear.x = 0.0;
                        cmd.angular.z = 0.0;
                        twist_cmd.linear.x = 0.0;
                        twist_cmd.angular.z = 0.0;
                    } else {
                        // 3초 경과 후 정지 해제
                        st->is_start_stop_active = false;
                        if (csv_index == cav_index) {
                            std::cout << "[CAV_index " << actual_cav_id << "] 3-second initial stop completed, starting motion" << std::endl;
                        }
                    }
                }
                
                // 정지 중이 아니면 정상 제어 진행
                if (!st->is_start_stop_active) {
                    // Check if ALL CAVs have finished 5 laps
                    if (mission_completed) {
                    // All CAVs finished 5 laps - STOP
                    cmd.linear.x  = 0.0;
                    cmd.angular.z = 0.0;
                    // twist_cmd.linear.x = -0.005; // Stop command
                    twist_cmd.linear.x = 0.0; // Stop command ->  바퀴 힘 없이 , 관성으로 랩타임 넘어가게끔,
                    twist_cmd.angular.z = 0.0;
                } else if (st->red_flag == 1) { 
                    // Red flag: STOP with -0.005
                    cmd.linear.x  = 0.0;
                    cmd.angular.z = 0.0;
                    twist_cmd.linear.x = -0.005; // Stop command
                    twist_cmd.angular.z = 0.0;
                // ***
                } else if (st->yellow_flag == 1) {
                    // Yellow flag Group 1 (Zone 1,2): Slow down to 0.8 m/s
                    cmd.linear.x  = 0.8;
                    twist_cmd.linear.x = 0.8;    //  < --------------1-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------2-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------3-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------4-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------5-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------6-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------7-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------8-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------9-------------------
                    // twist_cmd.linear.x = 0.8; //  < --------------10-------------------
                    cmd.angular.z = wz;
                    twist_cmd.angular.z = wz;
                } else if (st->yellow_flag == 2) {
                    // Yellow flag Group 2 (Zone 3,4,5,6,7,8): Slow down to 1.5 m/s
                    cmd.linear.x  = 0.8;
                    twist_cmd.linear.x = 0.8;      // < ---------------1-------------------
                    // twist_cmd.linear.x = 1.3;   //  < --------------2-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------3-------------------
                    // twist_cmd.linear.x = 1.3;   //  < --------------4-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------5-------------------
                    // twist_cmd.linear.x = 1.3;   //  < --------------6-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------7-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------8-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------9-------------------
                    // twist_cmd.linear.x = 1.0;   //  < --------------10-------------------
                    cmd.angular.z = wz;
                    twist_cmd.angular.z = wz;
                } else {
                    cmd.linear.x  = ramped_speed;
                    cmd.angular.z = wz;
                    twist_cmd.linear.x = ramped_speed;
                    twist_cmd.angular.z = wz;
                }
                    } // 정지 중이 아닐 때 제어 끝
                
                cmd.linear.y = 0.0; cmd.linear.z = 0.0;
                cmd.angular.x = 0.0; cmd.angular.y = 0.0;
                
                twist_cmd.linear.y = 0.0; twist_cmd.linear.z = 0.0;
                twist_cmd.angular.x = 0.0; twist_cmd.angular.y = 0.0;

                accel_pub->publish(cmd);    // simulator
                cmd_vel_pub->publish(twist_cmd); // motoir
            }
          }
      );
      pose_subs.push_back(sub); 

  }

  (void)flag_sub; (void)yellow_flag_sub; (void)vel_sub;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
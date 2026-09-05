/*
 * Mission 2 CAV 제어 노드의 진입점.
 * ROS2 초기화, Pub/Sub 설정, 그리고 Pose 수신마다 Mission -> Planning -> Control
 * 순서로 각 Process 함수를 호출하는 실행 흐름만 담당한다.
 */
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>
#include <std_msgs/msg/int32.hpp>

#include <chrono>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "Global/Global.hpp"
#include "Utils/Utils.hpp"
#include "Planning/Planning.hpp"
#include "Control/Control.hpp"
#include "Mission/Mission.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("overtake");
    auto st = std::make_shared<ControllerState>();
    auto st2 = std::make_shared<CavState>();

    // ---- CAV ID setup
    int cav_id = 1;
    const char* env_id = std::getenv("CAV_ID");
    if (env_id) {
        try {
            cav_id = std::stoi(env_id);
        } catch (...) {}
    }

    node->declare_parameter<int>("cav_id", cav_id);
    cav_id = node->get_parameter("cav_id").as_int();

    const std::string my_id_str = twoDigitId(cav_id);
    const std::string pose_topic = "/CAV_" + my_id_str;
    const std::string accel_topic = "/CAV_" + my_id_str + "_accel";
    const std::string red_flag_topic = "/CAV_" + my_id_str + "_RED_FLAG";

    RCLCPP_INFO(node->get_logger(),
                "[INIT] CAV Mission 2 RE-REFACTORED started (CAV_%s)", my_id_str.c_str());

    // ---- Control parameters
    node->declare_parameter<double>("speed_mps", 1.5);
    node->declare_parameter<double>("lookahead_m", 0.3);
    node->declare_parameter<double>("max_yaw_rate", 4.5);

    st->speed_mps = node->get_parameter("speed_mps").as_double();
    st->lookahead_m = node->get_parameter("lookahead_m").as_double();
    st->max_yaw_rate = node->get_parameter("max_yaw_rate").as_double();

    // ---- Preload all lane paths
    RCLCPP_INFO(node->get_logger(), "[PRELOAD] Loading all lanes...");
    for (int lane = 1; lane <= 3; ++lane) {
        std::string csv_path = std::string("/root/TEAM_AIM/src/global_path/") +
                               "path_mission2_" + std::to_string(cav_id) + "_" + twoDigitId(lane) + ".csv";

        if (!loadPathCsv(csv_path, lane_paths[lane])) {
            RCLCPP_ERROR(node->get_logger(), "[PRELOAD] Failed to load lane %d", lane);
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] Lane %d loaded: %zu points",
                    lane, lane_paths[lane].size());
    }

    // HV 경로 로드
    RCLCPP_INFO(node->get_logger(), "[PRELOAD] Loading HV paths for velocity measurement...");

    // HV20 (Lane 3) 경로
    std::string hv20_path = "/root/TEAM_AIM/src/global_path/path_mission2_1_20.csv";
    if (loadPathCsv(hv20_path, hv_paths[20])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV20 path loaded: %zu points", hv_paths[20].size());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV20 path");
    }

    // HV24 (Lane 2) 경로
    std::string hv24_path = "/root/TEAM_AIM/src/global_path/path_mission2_1_24.csv";
    if (loadPathCsv(hv24_path, hv_paths[24])) {
        RCLCPP_INFO(node->get_logger(), "[PRELOAD] HV24 path loaded: %zu points", hv_paths[24].size());
    } else {
        RCLCPP_WARN(node->get_logger(), "[PRELOAD] Failed to load HV24 path");
    }

    // ---- Set initial path
    current_lane = 2;  // Start with CENTER lane
    integrate_path_vector = lane_paths[current_lane];
    lane_start_idx[current_lane] = 0;

    if (integrate_path_vector.empty()) {
        RCLCPP_FATAL(node->get_logger(), "[FATAL] Initial path is empty");
        rclcpp::shutdown();
        return 1;
    }

    // ---- Publishers & Subscribers
    auto accel_pub = node->create_publisher<geometry_msgs::msg::Accel>(accel_topic, rclcpp::SensorDataQoS());

    auto flag_sub = node->create_subscription<std_msgs::msg::Int32>(
        red_flag_topic, 10,
        [st](const std_msgs::msg::Int32::SharedPtr msg) {
            st->red_flag = msg->data;
        }
    );

    // ---- HV subscriptions
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> hv_subs;
    for (int hv_id = 19; hv_id <= 36; ++hv_id) {
        std::string hv_topic = "/HV_" + std::to_string(hv_id);
        auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
            hv_topic, rclcpp::SensorDataQoS(),
            [hv_id](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                hv_positions[hv_id] = {msg->pose.position.x, msg->pose.position.y};
            }
        );
        hv_subs.push_back(sub);
    }
    RCLCPP_INFO(node->get_logger(), "[INIT] Subscribed to HV_19~HV_36");


    // HV 속도 측정을 위한 별도 구독
    auto hv20_vel_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_20", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            MeasureHVVelocity(msg, 20, node);
        }
    );

    auto hv24_vel_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/HV_24", rclcpp::SensorDataQoS(),
        [node](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            MeasureHVVelocity(msg, 24, node);
        }
    );
    RCLCPP_INFO(node->get_logger(), "[INIT] HV velocity measurement enabled (HV20, HV24)");

    init_overlap_region();
    // Set initial path
    current_lane = 2;
    integrate_path_vector = lane_paths[current_lane];
    lane_start_idx[current_lane] = 0;

    // ---- CAV pose subscription and control loop
    static int pose_callback_count = 0;
    auto last_log_time = std::chrono::steady_clock::now();
    const int LOG_INTERVAL_MS = 2000; // 로그 주기 단축

    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic, rclcpp::SensorDataQoS(),
        [node, st, st2, accel_pub, cav_id, &last_log_time](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {

            pose_callback_count++;

            // ===== 1. Update CAV pose =====
            cav_x = msg->pose.position.x;
            cav_y = msg->pose.position.y;
            cav_z = msg->pose.position.z;
            cav_yaw = yawFromQuat(msg->pose.orientation, st->prev_yaw);

            if (st->has_prev) {
                const double dx = cav_x - st->prev_x;
                const double dy = cav_y - st->prev_y;
                if (std::hypot(dx, dy) > 1e-4) {
                    cav_yaw = std::atan2(dy, dx);
                }
            }
            st->prev_x = cav_x;
            st->prev_y = cav_y;
            st->prev_yaw = cav_yaw;
            st->has_prev = true;

            // 2) Mission: Zone 1~5 충돌 판단 및 차선별 정지 필요 여부 계산
            std::vector<bool> lane_collision;
            bool is_overlap_zone = false;
            bool stop_flag = MissionProcess(cav_id, lane_collision, is_overlap_zone, node);

            // 2-1) Mission: 완주(Race Over) 판정 (Planning과 데이터 의존성 없음)
            bool race_over = is_race_over(*st2);

            // 3) Planning: 다음 차선 선택 및 필요 시 차선 전환
            int next_lane = PlanningProcess(cav_id, lane_collision,
                                             zone3_collision_flag, zone4_collision_flag, zone5_collision_flag,
                                             zone_collision_flag, zone2_collision_flag,
                                             stop_flag, node);

            // TODO(cleanup candidate, not fixed in this pass): this block duplicates the
            // change_csv_state() call already performed inside PlanningProcess() above with
            // the identical gating condition. It was already present as a duplicate call in
            // the pre-refactor main branch code (not introduced by this restructuring), and
            // lane_start_distance below is computed but never used to gate anything there
            // either. Left intact to preserve original behavior exactly; a future
            // behavior-changing cleanup could remove this block once verified redundant.
            double lane_start_distance = 1e10;
            if (lane_start_idx[current_lane] >= 0 && lane_start_idx[next_lane] >= 0) {
                 // 목표 차선까지의 거리 계산이 아니라, 차선 변경 안정성을 위해
                 // 현재 위치가 경로상에 제대로 있는지 확인하는 용도
                 lane_start_distance = calculateDistance(
                     lane_paths[next_lane][lane_start_idx[next_lane]].x,
                     lane_paths[next_lane][lane_start_idx[next_lane]].y,
                     cav_x, cav_y
                 );
            }

            // 차선 변경 실행 (1.5m 이내에 목표 차선의 시작점이 있어야 변경 - 너무 멀면 오류 가능성 배제)
            if (!zone_collision_flag && !zone2_collision_flag && !stop_flag && next_lane != current_lane) {
                 change_csv_state(cav_id, next_lane, node);
            }

            // 상태 확인용 차선 로그 (LOG_INTERVAL_MS 주기로 출력)
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_time).count();

            if (elapsed >= LOG_INTERVAL_MS) {
                std::cout << "\n=== [Lane Status] (CAV" << cav_id << ") ===" << std::endl;
                std::cout << "CAV Pos: (" << std::fixed << std::setprecision(2) << cav_x << ", " << cav_y << ")" << std::endl;

                if (is_overlap_zone) {
                    std::cout << ">>> OVERLAP DETECTED! Forcing Lane 3 -> Lane 2 logic <<<" << std::endl;
                }

                for (int lane = 1; lane <= 3; ++lane) {
                    std::string status = lane_collision[lane] ? "X OCCUPIED" : "O FREE";
                    if (is_overlap_zone && lane == 3) status += " (Overlap Force)";
                    std::cout << "  Lane " << lane << " : " << status << std::endl;
                }
                std::cout << "-> Current: " << current_lane << " -> Next: " << next_lane << std::endl;
                last_log_time = now;
            }

            // 4) Control: 목표 속도 계획, Pure Pursuit, 우선순위 기반 명령 생성
            geometry_msgs::msg::Accel cmd;
            if (ControlProcess(*st, stop_flag, race_over, node, cmd)) {
                accel_pub->publish(cmd);
            }
        }
    );

    (void)flag_sub;
    (void)pose_sub;
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

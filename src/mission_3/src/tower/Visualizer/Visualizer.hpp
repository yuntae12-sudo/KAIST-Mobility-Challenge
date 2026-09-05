/*
 * Zone/ROI/CAV 경로/HV 위치를 RViz MarkerArray로 시각화하는 함수를 담는다.
 */
#ifndef MISSION_3_TOWER_VISUALIZER_HPP
#define MISSION_3_TOWER_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

// ==============================
// Visualizer Process
// ==============================
void publish_visualization(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
    double precollision_radius,
    double imminent_collision_radius,
    int visualization_lookahead);

#endif  // MISSION_3_TOWER_VISUALIZER_HPP

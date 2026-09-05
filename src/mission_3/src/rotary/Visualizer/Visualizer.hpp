/*
 * Yellow ROI 감속 구간을 RViz MarkerArray로 시각화하는 함수를 담는다.
 */
#ifndef MISSION_3_ROTARY_VISUALIZER_HPP
#define MISSION_3_ROTARY_VISUALIZER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

// ==============================
// Visualizer Process
// ==============================
void publish_yellow_zone_markers(std::shared_ptr<rclcpp::Node> node,
                                  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
                                  double yellow_roi_detection_radius);

#endif  // MISSION_3_ROTARY_VISUALIZER_HPP

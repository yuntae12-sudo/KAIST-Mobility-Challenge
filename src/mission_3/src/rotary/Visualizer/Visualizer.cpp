/*
 * Visualizer.hpp에 선언된 Yellow Zone 시각화 함수의 구현.
 */
#include "Visualizer/Visualizer.hpp"

#include <visualization_msgs/msg/marker.hpp>

#include "Global/Global.hpp"

// Yellow ROI 감속 구간을 원기둥 마커와 텍스트 라벨로 시각화한다.
void publish_yellow_zone_markers(std::shared_ptr<rclcpp::Node> node,
                                  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
                                  double yellow_roi_detection_radius) {
    auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();

    int marker_id = 0;
    for (const auto& [yellow_roi_id, yellow_roi_pose] : yellow_cav_rois) {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = node->now();
        marker.ns = "yellow_zones";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set position
        marker.pose.position.x = yellow_roi_pose.x;
        marker.pose.position.y = yellow_roi_pose.y;
        marker.pose.position.z = 0.0;

        // Set orientation (no rotation needed for cylinder)
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set scale (radius and height)
        marker.scale.x = yellow_roi_detection_radius * 2.0;  // diameter
        marker.scale.y = yellow_roi_detection_radius * 2.0;  // diameter
        marker.scale.z = 0.01;  // thin cylinder for top-down view

        // Set color: Yellow (RGB)
        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.3f;  // semi-transparent

        marker_array->markers.push_back(marker);

        // Add text label
        auto text_marker = visualization_msgs::msg::Marker();
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = node->now();
        text_marker.ns = "yellow_zone_labels";
        text_marker.id = marker_id++;
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::msg::Marker::ADD;

        text_marker.pose.position.x = yellow_roi_pose.x;
        text_marker.pose.position.y = yellow_roi_pose.y;
        text_marker.pose.position.z = 0.2;

        text_marker.pose.orientation.w = 1.0;
        text_marker.scale.z = 0.15;  // text size

        text_marker.color.r = 1.0f;
        text_marker.color.g = 1.0f;
        text_marker.color.b = 0.0f;
        text_marker.color.a = 1.0f;

        text_marker.text = "Yellow_ROI_" + std::to_string(yellow_roi_id);
        marker_array->markers.push_back(text_marker);
    }

    marker_pub->publish(*marker_array);
}

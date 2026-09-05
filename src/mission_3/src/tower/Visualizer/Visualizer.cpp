/*
 * Visualizer.hpp에 선언된 RViz 시각화 함수의 구현.
 */
#include "Visualizer/Visualizer.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "Global/Global.hpp"
#include "Planning/Planning.hpp"

// Zone/ROI/CAV 경로/HV 위치를 매 프레임 다시 그린다.
void publish_visualization(
    std::shared_ptr<rclcpp::Node> node,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub,
    double precollision_radius,
    double imminent_collision_radius,
    int visualization_lookahead) {

  visualization_msgs::msg::MarkerArray marker_array;

  //(A) 매 프레임 RViz에 남아있는 이전 Marker를 확실히 정리
  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = "map";
  clear.header.stamp = node->now();
  clear.ns = "clear_all";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear);

  // ========================================
  // 1. Zone 시각화 (Zone 1, 2, 3만)
  // ========================================
  for (const auto& [zone_id, zone_pos] : zones) {

    // Precollision Zone (노란색, 반투명)
    visualization_msgs::msg::Marker precollision_marker;
    precollision_marker.header.frame_id = "map";
    precollision_marker.header.stamp = node->now();
    precollision_marker.ns = "precollision_zones";
    precollision_marker.id = 100 + zone_id;
    precollision_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    precollision_marker.action = visualization_msgs::msg::Marker::ADD;

    precollision_marker.pose.position.x = zone_pos.x;
    precollision_marker.pose.position.y = zone_pos.y;
    precollision_marker.pose.position.z = 0.0;
    precollision_marker.pose.orientation.w = 1.0;

    precollision_marker.scale.x = precollision_radius * 2.0;
    precollision_marker.scale.y = precollision_radius * 2.0;
    precollision_marker.scale.z = 0.01;

    precollision_marker.color.r = 1.0;
    precollision_marker.color.g = 1.0;
    precollision_marker.color.b = 0.0;
    precollision_marker.color.a = 0.3;

    marker_array.markers.push_back(precollision_marker);

    // Imminent Zone (빨간색, 반투명)
    visualization_msgs::msg::Marker imminent_marker;
    imminent_marker.header.frame_id = "map";
    imminent_marker.header.stamp = node->now();
    imminent_marker.ns = "imminent_zones";
    imminent_marker.id = 200 + zone_id;
    imminent_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    imminent_marker.action = visualization_msgs::msg::Marker::ADD;

    imminent_marker.pose.position.x = zone_pos.x;
    imminent_marker.pose.position.y = zone_pos.y;
    imminent_marker.pose.position.z = 0.0;
    imminent_marker.pose.orientation.w = 1.0;

    imminent_marker.scale.x = imminent_collision_radius * 2.0;
    imminent_marker.scale.y = imminent_collision_radius * 2.0;
    imminent_marker.scale.z = 0.01;

    imminent_marker.color.r = 1.0;
    imminent_marker.color.g = 0.0;
    imminent_marker.color.b = 0.0;
    imminent_marker.color.a = 0.5;

    marker_array.markers.push_back(imminent_marker);
  }

  // ========================================
  // 2. ROI 시각화 (빨간색, 반투명, 작은 원)
  // ========================================
  for (const auto& [roi_id, roi_pos] : rois) {
    visualization_msgs::msg::Marker roi_marker;
    roi_marker.header.frame_id = "map";
    roi_marker.header.stamp = node->now();
    roi_marker.ns = "roi_zones";
    roi_marker.id = 300 + roi_id;
    roi_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    roi_marker.action = visualization_msgs::msg::Marker::ADD;

    roi_marker.pose.position.x = roi_pos.x;
    roi_marker.pose.position.y = roi_pos.y;
    roi_marker.pose.position.z = 0.0;
    roi_marker.pose.orientation.w = 1.0;

    roi_marker.scale.x = ROI_RADIUS * 2.0;
    roi_marker.scale.y = ROI_RADIUS * 2.0;
    roi_marker.scale.z = 0.01;

    roi_marker.color.r = 1.0;
    roi_marker.color.g = 0.0;
    roi_marker.color.b = 0.0;
    roi_marker.color.a = 0.6;  // 반투명

    marker_array.markers.push_back(roi_marker);

    // ROI 텍스트 라벨
    visualization_msgs::msg::Marker roi_text;
    roi_text.header.frame_id = "map";
    roi_text.header.stamp = node->now();
    roi_text.ns = "roi_labels";
    roi_text.id = 400 + roi_id;
    roi_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    roi_text.action = visualization_msgs::msg::Marker::ADD;

    roi_text.pose.position.x = roi_pos.x;
    roi_text.pose.position.y = roi_pos.y;
    roi_text.pose.position.z = 0.3;
    roi_text.pose.orientation.w = 1.0;

    roi_text.text = "ROI_" + std::to_string(roi_id);
    roi_text.scale.z = 0.25;

    roi_text.color.r = 1.0;
    roi_text.color.g = 0.0;
    roi_text.color.b = 0.0;
    roi_text.color.a = 1.0;

    marker_array.markers.push_back(roi_text);
  }

  // ========================================
  // 3. 각 CAV의 Lookahead 경로 시각화
  // ========================================
  for (int cav_id = 1; cav_id <= 4; cav_id++) {
    if (cav_poses.find(cav_id) == cav_poses.end()) continue;
    if (cav_paths[cav_id].empty()) continue;

    int current_idx = find_closest_waypoint_index(cav_paths[cav_id], cav_poses[cav_id]);
    std::vector<Pose> lookahead = get_lookahead_waypoints(cav_paths[cav_id], current_idx, visualization_lookahead);
    if (lookahead.empty()) continue;

    // LINE_STRIP 마커 생성
    visualization_msgs::msg::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = node->now();
    path_marker.ns = "lookahead_paths";
    path_marker.id = 1000 + cav_id;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::msg::Marker::ADD;

    path_marker.scale.x = 0.05;

    Color color = cav_colors[cav_id];
    path_marker.color.r = color.r;
    path_marker.color.g = color.g;
    path_marker.color.b = color.b;
    path_marker.color.a = color.a;

    path_marker.pose.orientation.w = 1.0;

    path_marker.points.clear();
    path_marker.points.reserve(lookahead.size());

    for (const auto& point : lookahead) {
      geometry_msgs::msg::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.1;
      path_marker.points.push_back(p);
    }

    marker_array.markers.push_back(path_marker);

    // CAV 위치에 텍스트 표시 (CAV ID)
    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = node->now();
    text_marker.ns = "cav_labels";
    text_marker.id = 2000 + cav_id;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;

    text_marker.pose.position.x = cav_poses[cav_id].x;
    text_marker.pose.position.y = cav_poses[cav_id].y;
    text_marker.pose.position.z = 0.6;
    text_marker.pose.orientation.w = 1.0;

    text_marker.text = "CAV_" + std::to_string(cav_id);
    text_marker.scale.z = 0.35;

    text_marker.color.r = color.r;
    text_marker.color.g = color.g;
    text_marker.color.b = color.b;
    text_marker.color.a = 1.0;

    marker_array.markers.push_back(text_marker);
  }

  for (const auto& [hv_id, hv_pose] : hv_poses) {
  visualization_msgs::msg::Marker hv_marker;
  hv_marker.header.frame_id = "map";
  hv_marker.header.stamp = node->now();
  hv_marker.ns = "hv_positions";
  hv_marker.id = 3000 + hv_id;
  hv_marker.type = visualization_msgs::msg::Marker::SPHERE;
  hv_marker.action = visualization_msgs::msg::Marker::ADD;

  hv_marker.pose.position.x = hv_pose.x;
  hv_marker.pose.position.y = hv_pose.y;
  hv_marker.pose.position.z = 0.15;
  hv_marker.pose.orientation.w = 1.0;

  hv_marker.scale.x = 0.3;
  hv_marker.scale.y = 0.3;
  hv_marker.scale.z = 0.3;

  hv_marker.color.r = 1.0;
  hv_marker.color.g = 0.0;
  hv_marker.color.b = 0.0;
  hv_marker.color.a = 1.0;

  marker_array.markers.push_back(hv_marker);
}


marker_pub->publish(marker_array);
}

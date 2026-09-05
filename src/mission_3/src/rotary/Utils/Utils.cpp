/*
 * Utils.hpp에 선언된 범용 헬퍼 함수들의 구현.
 */
#include "Utils/Utils.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

// 두 위치 사이의 유클리드 거리를 계산한다.
double calculate_distance(Pose pose1, Pose pose2) {
    double dx = pose1.x - pose2.x;
    double dy = pose1.y - pose2.y;
    return std::hypot(dx, dy);
}

// 원형 교차로 중심(cx, cy)을 기준으로 두 점 사이의 호 길이를 계산한다.
double GetArcLength(double x1, double y1, double x2, double y2,
                      double cx, double cy) {
    double r1 = std::hypot(x1 - cx, y1 - cy);
    double r2 = std::hypot(x2 - cx, y2 - cy);
    double radius = (r1 + r2) / 2.0;

    double theta1 = atan2(y1 - cy, x1 - cx);
    double theta2 = atan2(y2 - cy, x2 - cx);

    double delta_theta = fabs(theta2 - theta1);


    double arc = radius * delta_theta;
    return arc;
}

// x,y 좌표 쌍으로 이루어진 경로 CSV 파일을 읽어 온다. 헤더 줄("x,y")은 건너뛴다.
std::vector<Pose> load_csv_file(const std::string& file_path) {
  std::vector<Pose> csv_data;
  std::ifstream file(file_path);

  if (!file.is_open()) {
    std::cerr << "[WARN] Failed to open: " << file_path << std::endl;
    return csv_data;
  }

  std::string line;
  int line_num = 0;

  while (std::getline(file, line)) {
    line_num++;

    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }

    if (line_num == 1 && (line.find("x,y") != std::string::npos ||
                          line.find("X,Y") != std::string::npos)) {
      continue;
    }

    std::stringstream ss(line);
    std::string x_str, y_str;

    if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
      try {
        x_str.erase(0, x_str.find_first_not_of(" \t\r\n"));
        x_str.erase(x_str.find_last_not_of(" \t\r\n") + 1);
        y_str.erase(0, y_str.find_first_not_of(" \t\r\n"));
        y_str.erase(y_str.find_last_not_of(" \t\r\n") + 1);

        Pose pose;
        pose.x = std::stod(x_str);
        pose.y = std::stod(y_str);
        csv_data.push_back(pose);
      } catch (...) {
        continue;
      }
    }
  }

  return csv_data;
}

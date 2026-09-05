/*
 * Global.hpp에 선언된 위치(Localization) 전역 상태의 정의와
 * PoseStamped 메시지에서 좌표/자세를 추출하는 함수를 담는다.
 */
#include "Global/Global.hpp"

double x_m = 0.0;
double y_m = 0.0;
double z_m = 0.0;

double x_q = 0.0;
double y_q = 0.0;
double z_q = 0.0;
double w_q = 0.0;

int closest_index = 0;

// PoseStamped 메시지에서 위치와 쿼터니언 자세 값을 꺼내온다.
void get_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg,
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

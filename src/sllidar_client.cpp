/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <cmath>

#include "opencv2/opencv.hpp"
using namespace std;
using namespace cv;

#define RAD2DEG(x) ((x)*180./M_PI)

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);
  printf("[SLLIDAR INFO]: angle_range : [%f, %f]\n", RAD2DEG(scan->angle_min),
         RAD2DEG(scan->angle_max));
  Mat canvas(Size(500,500), CV_8UC3, Scalar(255,255,255));
  Mat rot = getRotationMatrix2D(Point(250,250), 0.5, 1.0);
  float distance = 25.0f;
  for (int i = 0; i < count; i++) {
    float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
    printf("count : %d , [SLLIDAR INFO]: angle-distance : [%f, %f]\n", count , degree, scan->ranges[i]);
    float x = 250 + distance * scan->ranges[i] * cos(degree * CV_PI / 180.0f);
    float y = 250 + distance * scan->ranges[i] * sin(degree * CV_PI / 180.0f);
    drawMarker(canvas, Point(cvRound(x),cvRound(y)),Scalar(0,0,255),MARKER_SQUARE,2);
    
  }
  imshow("win", canvas);
  waitKey(1);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}

#pragma once

#include "sensor_msgs/LaserScan.h"

class LidarScan {
public:
  sensor_msgs::LaserScan rawScanData;
  float wallDist;
  float wallAngle;

public:
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
};

#include "ros/console.h"
#include <contest2/contest2.h>
#include <contest2/lidar.h>

void LidarScan::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  int startIdx = 0;
  int endIdx = msg->ranges.size() - 1;
  int minIdx = -1;

  float checkAngle = msg->angle_min;

  this->wallDist = 20;
  for (int rangeIdx = 0; rangeIdx < endIdx; rangeIdx++) {
    if (std::isfinite(msg->ranges[rangeIdx]) &&
        msg->ranges[rangeIdx] >= msg->range_min &&
        msg->ranges[rangeIdx] <= msg->range_max) {

      // Check if the detection angle is within the cone we want
      checkAngle = RAD2DEG(msg->angle_min + rangeIdx * msg->angle_increment);
      if (checkAngle >= std::get<0>(ANGLE_CONE) &&
          checkAngle <= std::get<1>(ANGLE_CONE)) {
        this->wallDist = std::min(this->wallDist, msg->ranges[rangeIdx]);
        minIdx = rangeIdx;
      }
    };
  };

  if (wallDist == 20) {
    ROS_WARN("Could not find valid LIDAR value.");
    wallDist = 0;
  };

  wallAngle = RAD2DEG(msg->angle_min + minIdx * msg->angle_increment);
  rawScanData = *msg;
}

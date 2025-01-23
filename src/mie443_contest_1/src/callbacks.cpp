#include <iostream>

#include "contest1.h"

StateVars::StateVars() {
  posX = 0.0;
  posY = 0.0;
  yaw = 0.0;

  wallDist = std::numeric_limits<float>::max();
  wallAngle = 0;

  bumper[0] = bumper[1] = bumper[2] = kobuki_msgs::BumperEvent::RELEASED;
}

void StateVars::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  bumper[msg->bumper] = msg->state;
}

void StateVars::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  int startIndex = 0;
  int endIndex = msg->ranges.size() - 1;
  int minIndex = -1;
  for (int range = 0; range < endIndex; range++) {
    if (std::isfinite(msg->ranges[range]) &&
        msg->ranges[range] >= msg->range_min &&
        msg->ranges[range] <= msg->range_max) {
      wallDist = std::min(wallDist, msg->ranges[range]);
      minIndex = range;
    };
  };
  wallAngle = RAD2DEG(msg->angle_min + minIndex * msg->angle_increment);
}

void StateVars::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));
}

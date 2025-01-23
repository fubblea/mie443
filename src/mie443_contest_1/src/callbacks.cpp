#include <iostream>

#include "contest1.h"

uint8_t bumper[NUM_BUMPERS] = {kobuki_msgs::BumperEvent::RELEASED,
                               kobuki_msgs::BumperEvent::RELEASED,
                               kobuki_msgs::BumperEvent::RELEASED};

// CALLBACK FUNCTIONS
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  bumper[msg->bumper] = msg->state;
}

float wallDist = std::numeric_limits<float>::max();
float wallAngle = 0;
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // std::cout<<*msg<<std::endl;
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

// Odometer variables
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));
}

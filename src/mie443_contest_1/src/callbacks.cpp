#include "contest1.h"

StateVars::StateVars() {
  posX = 0.0;
  posY = 0.0;
  yaw = 0.0;

  wallDist = std::numeric_limits<float>::max();
  wallAngle = 0;

  bumper[0] = bumper[1] = bumper[2] = kobuki_msgs::BumperEvent::RELEASED;
  bumperHit = BumperHit::NOTHING;
}

void StateVars::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  bumper[msg->bumper] = msg->state;
}

void StateVars::laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  int startIdx = 0;
  int endIdx = msg->ranges.size() - 1;
  int minIdx = -1;

  float checkAngle = msg->angle_min;

  wallDist = 20;
  for (int rangeIdx = 0; rangeIdx < endIdx; rangeIdx++) {
    if (std::isfinite(msg->ranges[rangeIdx]) &&
        msg->ranges[rangeIdx] >= msg->range_min &&
        msg->ranges[rangeIdx] <= msg->range_max) {

      // Check if the detection angle is within the cone we want
      checkAngle = RAD2DEG(msg->angle_min + rangeIdx * msg->angle_increment);
      if (checkAngle >= std::get<0>(ANGLE_CONE) &&
          checkAngle <= std::get<1>(ANGLE_CONE)) {
        wallDist = std::min(wallDist, msg->ranges[rangeIdx]);
        minIdx = rangeIdx;
      }
    };
  };

  if (wallDist == 20) {
    ROS_WARN("Could not find valid LIDAR value.");
    wallDist = 0;
  };

  wallAngle = RAD2DEG(msg->angle_min + minIdx * msg->angle_increment);
}

void StateVars::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));
}

void StateVars::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
  ROS_INFO("Reading map at: %f", msg->info.resolution);
}

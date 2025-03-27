#include "contest3/callbacks.h"
#include "contest3/contest3.h"
#include "visualization_msgs/Marker.h"
#include <contest3/state.h>

RobotPose::RobotPose() {
  posX = 0.0;
  posY = 0.0;
  yaw = 0.0;
}

void RobotState::followerCB(const geometry_msgs::Twist::ConstPtr &msg) {
  this->follow_cmd = *msg;
}

void RobotState::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  this->bumper[msg->bumper] = msg->state;
}

void RobotState::followerMarkerCB(
    const visualization_msgs::Marker::ConstPtr &msg) {
  this->follow_marker = *msg;
}

void RobotState::cliffCB(const kobuki_msgs::CliffEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  this->cliffEvent = *msg;
}

void RobotPose::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  posX = msg->pose.pose.position.x;
  posY = msg->pose.pose.position.y;
  yaw = RAD2DEG(tf::getYaw(msg->pose.pose.orientation));
}

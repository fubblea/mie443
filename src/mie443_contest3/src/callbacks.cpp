#include <contest3/state.h>

void RobotState::followerCB(const geometry_msgs::Twist::ConstPtr &msg) {
  this->follow_cmd = *msg;
}

void RobotState::bumperCB(const kobuki_msgs::BumperEvent::ConstPtr &msg) {
  // Get the state of the bumper that is pressed
  // access using bumper[kobuki_msgs::BumperEvent::{}] LEFT, CENTER, or RIGHT
  this->bumper[msg->bumper] = msg->state;
}

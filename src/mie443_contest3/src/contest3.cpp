#include "contest3/contest3.h"
#include "contest3/state.h"
#include <chrono>
#include <contest3/header.h>
#include <contest3/imageTransporter.hpp>
#include <ros/package.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "contest3");
  ros::NodeHandle nh;
  sound_play::SoundClient sc;
  teleController eStop;

  RobotState robotState(sc);

  // publishers
  ros::Publisher vel_pub =
      nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

  // subscribers
  ros::Subscriber follower =
      nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10,
                   &RobotState::followerCB, &robotState);
  ros::Subscriber follower_marker =
      nh.subscribe("turtlebot_follower/marker", 10,
                   &RobotState::followerMarkerCB, &robotState);
  ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10,
                                        &RobotState::bumperCB, &robotState);

  // contest count down timer
  ros::Rate loop_rate(10);
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  uint64_t secondsElapsed = 0;

  imageTransporter rgbTransport(
      "camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
  // imageTransporter rgbTransport("camera/rgb/image_raw",
  // sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
  imageTransporter depthTransport("camera/depth_registered/image_raw",
                                  sensor_msgs::image_encodings::TYPE_32FC1);

  while (ros::ok() && secondsElapsed <= 480) {
    ros::spinOnce();

    robotState.updateState(secondsElapsed, CONTEST_MODE);
    vel_pub.publish(robotState.getVelCmd());

    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now() - start)
                         .count();
    loop_rate.sleep();
  }

  return 0;
}

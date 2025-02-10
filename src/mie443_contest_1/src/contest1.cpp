#include "contest1.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  robotState state; // Initialize robot state machine

  ros::Subscriber bumper_sub =
      nh.subscribe("mobile_base/events/bumper", 10, &StateVars::bumperCallback,
                   &state.stateVars);
  ros::Subscriber laser_sub =
      nh.subscribe("scan", 10, &StateVars::laserCallback, &state.stateVars);
  ros::Subscriber odom_sub =
      nh.subscribe("odom", 1, &StateVars::odomCallback, &state.stateVars);

  ros::Publisher vel_pub =
      nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

  ros::Rate loop_rate(20); // Processing frequency [Hz]

  geometry_msgs::Twist vel;

  // Contest count down timer
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  uint64_t secondsElapsed = 0;

  while (ros::ok() && secondsElapsed <= 480) {
    ros::spinOnce();

    state.update();

    vel.angular.z = DEG2RAD(state.getVelCmd().angular);
    vel.linear.x = state.getVelCmd().linear;
    vel_pub.publish(vel);

    // The last thing to do is to update the timer.
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now() - start)
                         .count();
    loop_rate.sleep();
  }

  ROS_ERROR("I'm outta time!!!!");
  ROS_INFO("Map saved");
  system("rosrun map_server map_saver -f logs/");
  return 0;
}

#include "contest1.h"

// Class constructors

Vel::Vel(float angular, float linear) {
  angular = angular;
  linear = linear;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  robotState state;                 // Initialize robot state machine
  tf::TransformListener tfListener; // Init transform listener

  // Set Params
  nh.setParam("/slam_gmapping/map_update_interval", 0.03);

  // Subscribers
  ros::Subscriber bumper_sub =
      nh.subscribe("mobile_base/events/bumper", 10, &StateVars::bumperCallback,
                   &state.stateVars);
  ros::Subscriber laser_sub =
      nh.subscribe("scan", 10, &StateVars::laserCallback, &state.stateVars);
  ros::Subscriber odom_sub =
      nh.subscribe("odom", 1, &StateVars::odomCallback, &state.stateVars);
  ros::Subscriber map_sub =
      nh.subscribe("map", 1, &StateVars::mapCallback, &state.stateVars);

  // Publishers
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

    state.update(tfListener);

    vel.angular.z = DEG2RAD(state.getVelCmd().angular);
    vel.linear.x = state.getVelCmd().linear;
    vel_pub.publish(vel);

    ROS_INFO("Map Pose: (%f, %f, %f)", state.stateVars.mapPose.posX,
             state.stateVars.mapPose.posY, state.stateVars.mapPose.yaw);

    // The last thing to do is to update the timer.
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now() - start)
                         .count();
    loop_rate.sleep();
  }

  ROS_ERROR("I'm outta time!!!!");

  return 0;
}

#include "contest1.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"

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

  // Advertise additional topics for debugging
  ros::Publisher goal_pub =
      nh.advertise<geometry_msgs::PointStamped>("robot_debug/robot_goal", 1);
  ros::Publisher path_pub =
      nh.advertise<nav_msgs::Path>("robot_debug/robot_path", 1);
  ros::Publisher inflatedMap_pub =
      nh.advertise<nav_msgs::OccupancyGrid>("robot_debug/inflated_map", 1);

  ros::Rate loop_rate(20); // Processing frequency [Hz]

  geometry_msgs::Twist vel;

  // Contest count down timer
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  uint64_t secondsElapsed = 0;

  while (ros::ok() && secondsElapsed <= 480) {
    // These are reset every loop
    geometry_msgs::PointStamped goal;
    nav_msgs::Path path;

    ros::spinOnce();

    state.update(tfListener);

    // Publish velocity
    vel.angular.z = DEG2RAD(state.getVelCmd().angular);
    vel.linear.x = state.getVelCmd().linear;
    vel_pub.publish(vel);

    // Publish goal
    goal.header.stamp = ros::Time::now();
    goal.header.frame_id = state.stateVars.map.header.frame_id;
    goal.point.x = state.stateVars.goal.posX;
    goal.point.y = state.stateVars.goal.posY;
    goal_pub.publish(goal);

    // Publish the path
    path.header.stamp = ros::Time::now();
    path.header.frame_id = state.stateVars.map.header.frame_id;
    for (std::tuple<float, float> point : state.stateVars.pathPoints) {
      geometry_msgs::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = std::get<0>(point);
      pose.pose.position.y = std::get<1>(point);
      pose.pose.position.z = 0;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub.publish(path);

    // Publish the inflated map
    inflatedMap_pub.publish(state.stateVars.inflatedMap);

    // The last thing to do is to update the timer.
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(
                         std::chrono::system_clock::now() - start)
                         .count();
    loop_rate.sleep();
  }

  ROS_ERROR("I'm outta time!!!!");

  return 0;
}

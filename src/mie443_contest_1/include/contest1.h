#ifndef CONTEST1_H
#define CONTEST1_H

// EXTERNAL HEADER FILES

#include "ros/ros.h"
#include <algorithm>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <stdio.h>
#include <tuple>
#include <vector>

#include <chrono>

// CONSTANTS

const float MAX_LIN_VEL = 0.25; // Maximum linear velocity in [m/s]
const float SLOW_LIN_VEL = 0.1; // Slow linear velocity in [m/s]

const float MAX_SPIN_ANGLE = 355; // Maximum spin angle in [deg]

const float MAX_ANG_VEL = 25; // Maximum angular velocity in [deg/s]
const float MIN_ANG_VEL = 5;  // Minimum angular velocity in [deg/s]
const float ANGLE_TOL = 1;    // Angle error tolerance in [deg]

const int NUM_BUMPERS = 3; // Number of bumpers

const float MIN_WALL_DIST =
    0.46; // Minimum wall distance that the lidar can detect in [m]
const std::tuple<float, float> ANGLE_CONE =
    std::make_tuple(-20, 20); // Angle cone for lidar detection[deg]

const int MAP_SEARCH_WIDTH = 1000; // Costmap search width
const int MAP_SEARCH_DEPTH = 1000; // Costmap search depth
const int UNKNOWN_WEIGHT = -50; // Weight to assign to squares that are unknown

const float SPACE_WEIGHT = 1.2; // Weight to assign to available space
const float KNOWN_WEIGHT = 0;   // Weight to assign to known areas

// Frontier goal setting consts
const int START_SEARCH_SIZE = 10;
const int MAX_IN_BOX_SEARCHES = 100;
const int MAX_SEARCH_ATTEMPTS = 10;
const int MAX_EXCLUSIONS = 10;

// Path mapping consts
const int CELL_OCCUPANCY_THRESH = 1;
const int NEIGHBOR_COST = 1;
const float WAYPOINT_DIST_TOL = 0.1;
const int OBSTACLE_PADDING = 10;
const int MAX_MOVE_ATTEMPTS = 3;

// CLASS DEFS

/*
Angular and linear velocity
*/
class Vel {
public:
  float angular = 0.0; // Angular velocity [deg/s]
  float linear = 0.0;  // Linear velocity [m/s]

  // Constructor
  Vel();
  Vel(float angular, float linear);
};

/*
Robot pose
*/
class Pose {
public:
  float posX = 0; // X position relative to frame origin [m]
  float posY = 0; // Y position relative to frame origin [m]
  float yaw = 0;  // Yaw angle [deg]

  Pose() {
    posX = 0;
    posY = 0;
    yaw = 0;
  }
  Pose(float posX, float posY) {
    posX = posX;
    posY = posY;
    yaw = 0;
  }
};

// INTERNAL HEADER FILES

#include <callbacks.h>
#include <pathfind.h>
#include <state.h>

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

#endif // CONTEST1_H

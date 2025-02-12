#ifndef CONTEST1_H
#define CONTEST1_H

// EXTERNAL HEADER FILES

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>

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

const float VISITED_BOX = 1; // Size of the box [m]

// Wall following params
const float WALL_FOLLOW_TIME = 0;
const int FRONT_DETECT_RANGE = 50; // In samples
const int SIDE_DETECT_RANGE = 5;   // Positive values are left side. In samples
const float WALL_FOLLOW_DIST = 0.5;
const float CONTROLLER_KP = 5;

// INTERAL HEADER FILES

#include <callbacks.h>
#include <state.h>

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

#endif // CONTEST1_H

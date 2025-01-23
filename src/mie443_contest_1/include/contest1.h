#ifndef CONTEST1_H
#define CONTEST1_H

// EXTERNAL HEADER FILES

#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <cmath>

#include <chrono>

// INTERAL HEADER FILES

#include <callbacks.h>
#include <state.h>

// MACROS

#define N_BUMPER (3) // Number of bumpers
#define RAD2DEG(rad) ((rad) *180./M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) *M_PI /180.) // Convert from degrees to radians

#endif // CONTEST1_H

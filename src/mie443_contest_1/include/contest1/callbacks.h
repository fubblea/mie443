#pragma once

#include "contest1/contest1.h"

/*
Possible bumper hit states
*/
enum BumperHit { LEFT, CENTER, RIGHT, NOTHING };

/*
Variables that represent the current robot state
*/
class StateVars {
public:
  // Odometer variables
  float posX; // X position relative to start [m]
  float posY; // Y position relative to start [m]
  float yaw;  // Yaw angle [deg]

  // Lidar Variables
  float wallDist;                     // Distance to closest wall [m]
  float wallAngle;                    // Angle to closest wall [deg]
  sensor_msgs::LaserScan rawScanData; // Raw lidar scan data

  // Bumper Variables
  uint8_t bumper[NUM_BUMPERS]; // Bumper states

  // Vector of previously visited positions
  std::vector<std::tuple<float, float>> visitedPos;

  // The bumper that is hit
  BumperHit bumperHit;

  // Space either side of the robot. Used in CHECK_RIGHT/CHECK_LEFT
  std::tuple<float, float> sideSpace;

  // Check if a certain side has been visited previously
  std::tuple<bool, bool> sideVisited;

  // Wall following variables
  float frontWallDist;
  float sideWallDist;

  // Construct with default values
  StateVars();

  // Callbacks
  /*
  Callback for the turtlebot bumper
  */
  void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg);

  /*
  Callback for the turtlebot lidar
  */
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

  /*
  Callback for the turtlebot odometer
  */
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

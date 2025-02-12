#ifndef CALLBACKS_H
#define CALLBACKS_H

#include "contest1.h"
#include <vector>

/*
Possible bumper hit states
*/
enum BumperHit { LEFT, CENTER, RIGHT, NOTHING };

/*
Possible robot states.
*/
enum State {
  START,
  SPIN,
  THINK,
  IM_HIT,
  IM_CHECKING,
  IM_SPEED,
  CHECK_RIGHT,
  CHECK_LEFT,
  DO_MATH,
  REORIENT_GOAL,
  REORIENT_SPACE,
  END
};

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
  float wallDist;  // Distance to closest wall [m]
  float wallAngle; // Angle to closed wall [deg]

  // Bumper Variables
  uint8_t bumper[NUM_BUMPERS]; // Bumper states

  // Vector of previously visited positions
  std::vector<std::tuple<float, float>> visitedPos;

  // The bumper that is hit
  BumperHit bumperHit;

  // Space either side of the robot. Used in CHECK_RIGHT/CHECK_LEFT
  std::tuple<float, float> sideSpace;

  // Map variables
  nav_msgs::OccupancyGrid map;    // Costmap
  Pose mapPose;                   // Robot pose wrt map frame
  std::tuple<int, int> gridIdx;   // Indexes of the robot on the costmap
  std::tuple<int, int> sideKnown; // Point value of how mapped each side is.
                                  // (Higher value means more is mapped)

  std::tuple<float, float> sideScore; // Weighted score of each side

  // Variables for bigger brain strat
  Pose goal;
  std::tuple<int, int> goalIdx;
  std::vector<std::tuple<float, float>> excludedPoints;
  int excludeAttempts = 0;

  // Variables for path following
  std::vector<std::tuple<float, float>> pathPoints;
  Pose wayPoint;
  int moveAttemps = 0;

  State oldState;

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

  /*
  Callback for the turtlebot occupancy grid
  */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
};

#endif // CALLBACKS_H

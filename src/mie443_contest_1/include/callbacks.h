#ifndef CALLBACKS_H
#define CALLBACKS_H

// CALLBACK FUNCTIONS

/* 
Callback for the turtlebot bumper
*/
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);

/* 
Callback for the turtlebot lidar
*/
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

/* 
Callback for the turtlebot odometer
*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

// CALLBACK VARIABLES

extern float posX; // X position relative to start [m]
extern float posY; // Y position relative to start [m]
extern float yaw; // Yaw angle [deg]
extern float wallDist; // Distance to closest wall [m]
extern float wallAngle; // Angle to closed wall [deg]
extern uint8_t bumper[3]; // Bumper states

#endif // CALLBACKS_H

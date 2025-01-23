#ifndef CALLBACKS_H
#define CALLBACKS_H

// CALLBACK FUNCTIONS
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

// CALLBACK VARIABLES
extern float posX;
extern float posY;
extern float yaw; // deg
extern float wallDist;
extern float wallAngle; // deg
extern uint8_t bumper[3];
#endif // CALLBACKS_H

#include "contest1.h"

// CALLBACK FUNCTIONS
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;
}

// Odometer variables
float posX = 0.0;
float posY = 0.0;
float yaw = 0.0;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
}

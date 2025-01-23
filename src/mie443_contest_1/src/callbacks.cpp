#include "contest1.h"

// CALLBACK FUNCTIONS
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;
}

float minimumDistance = std::numeric_limits<float>::max();
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	// std::cout<<*msg<<std::endl;
    int startIndex = 0;
    int endIndex = msg->ranges.size()-1;
    for(int range =0; range < endIndex; range++){
        if(std::isfinite(msg->ranges[range])&& msg->ranges[range] >= msg->range_min && msg->ranges[range] <= msg->range_max){
            minimumDistance = std::min(minimumDistance, msg->ranges[range]);
        };
    };
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

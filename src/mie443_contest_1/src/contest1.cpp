#include "contest1.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;

    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 1, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 1, &laserCallback);
    ros::Subscriber odom_sub = nh.subscribe("odom", 1, &odomCallback);

    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

    ros::Rate loop_rate(100); // Processing frequency [Hz]

    geometry_msgs::Twist vel;

    // contest count down timer
    std::chrono::time_point<std::chrono::system_clock> start;
    start = std::chrono::system_clock::now();
    uint64_t secondsElapsed = 0;

    robotState state;

    while(ros::ok() && secondsElapsed <= 480) {
        ros::spinOnce();

        state.step();

        vel.angular.z = state.getVelCmd().angular;
        vel.linear.x = state.getVelCmd().linear;
        vel_pub.publish(vel);

        //ROS_INFO("Position: (%f, %f). Yaw: %f", posX, posY, yaw);
        ROS_INFO("Minimum Distance: (%f, %f)", minimumDistance);

        // The last thing to do is to update the timer.
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now()-start).count();
        loop_rate.sleep();
    }

    return 0;
}

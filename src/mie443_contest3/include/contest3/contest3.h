#pragma once

#include "ros/package.h"
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <string>
#include <tf/transform_datatypes.h>

const bool CONTEST_MODE = true; // True for real life, false if not.

const int GO_HOME_TIME = 480;
const int NUM_BUMPERS = 3;

const float MAX_ANGLE_VEL = 10;
const float MIN_ANGLE_VEL = 5;
const float ANGLE_TOL = 1;

const std::string SOUND_PATHS =
    ros::package::getPath("mie443_contest3") + "/sounds/";

const std::string IMG_PATHS =
    ros::package::getPath("mie443_contest3") + "/images/";

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

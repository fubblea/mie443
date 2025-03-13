#ifndef CONTEST2_H
#define CONTEST2_H

// EXTERNAL HEADER FILES

#include <chrono>

#include <cmath>
#include <stdio.h>
#include <tuple>
#include <vector>

#include <chrono>

#include <opencv/cv.h>
#include <ros/package.h>

using namespace cv;

// CONSTANTS
// TODO: Check if this is valid
const bool CONTEST_MODE = false; // false for test, true for contest

const float GO_HOME_TIME = 230;

const float MAX_LIN_VEL = 0.25; // Maximum linear velocity in [m/s]
const float SLOW_LIN_VEL = 0.1; // Slow linear velocity in [m/s]

const float MAX_SPIN_ANGLE = 340; // Maximum spin angle in [deg]

const float MAX_ANG_VEL = 25; // Maximum angular velocity in [deg/s]
const float MIN_ANG_VEL = 5;  // Minimum angular velocity in [deg/s]
const float ANGLE_TOL = 5;    // Angle error tolerance in [deg]

const float MIN_WALL_DIST =
    0.46; // Minimum wall distance that the lidar can detect in [m]
const std::tuple<float, float> ANGLE_CONE =
    std::make_tuple(-25, 25); // Angle cone for lidar detection[deg]

const int NUM_BUMPERS = 3; // Number of bumpers

const float BOX_FACING_OFFSET = 0.6;
const float BOX_ANGLE_OFFSET = 10;

const int MAX_LOST_COUNT = 1;

const int MIN_HESSIAN = 1000;
const float MATCH_COMPARE_THRESH = 0.65;
const int MIN_ROWS_BLANK = 50; // treshold to determine blank

const bool MANUAL_CROP = false;
const int MANUAL_CROP_X = 8;
const int MANUAL_CROP_Y = 3;

const bool GAUSSIAN_CROP = true;
const cv::Size CROP_SIZE = cv::Size(5, 5);
const float MIN_CROP_THRESH = 150;

// Paths to the template files
const std::vector<std::string> TEMPLATE_FILES = {
    ros::package::getPath("mie443_contest2") +
        std::string("/boxes_database/cinnamon_toast_crunch.jpg"),
    ros::package::getPath("mie443_contest2") +
        std::string("/boxes_database/raisin_bran.jpg"),
    ros::package::getPath("mie443_contest2") +
        std::string("/boxes_database/rice_krispies.jpg")};

// INTERAL HEADER FILES

#include <contest2/boxes.h>
#include <contest2/imagePipeline.h>
#include <contest2/lidar.h>
#include <contest2/navigation.h>
#include <contest2/robot_pose.h>
#include <contest2/state.h>

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

#endif // CONTEST2_H

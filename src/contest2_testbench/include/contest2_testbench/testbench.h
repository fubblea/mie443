#pragma once

// External header files

#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/package.h"
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <std_msgs/String.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <iostream>
#include <vector>

// Constants

const std::string IMAGE_DATABASE_PATH =
    ros::package::getPath("contest2_testbench") +
    std::string("/image_database/");

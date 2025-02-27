#ifndef CONTEST2_H
#define CONTEST2_H

// EXTERNAL HEADER FILES

#include <boxes.h>
#include <chrono>
#include <imagePipeline.h>
#include <navigation.h>
#include <robot_pose.h>

#include <cmath>
#include <stdio.h>
#include <tuple>
#include <vector>

#include <chrono>

// CONSTANTS

const float BOX_FACING_OFFSET = 0.5;

// INTERAL HEADER FILES

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

#endif // CONTEST2_H

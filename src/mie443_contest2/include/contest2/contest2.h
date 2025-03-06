#ifndef CONTEST2_H
#define CONTEST2_H

// EXTERNAL HEADER FILES

#include <chrono>

#include <cmath>
#include <stdio.h>
#include <tuple>
#include <vector>

#include <chrono>

// CONSTANTS

// TODO: Check if this is valid
const bool CONTEST_MODE = true; // 0 for test, 1 for contest

const float BOX_FACING_OFFSET = 0.7;
const float BOX_ANGLE_OFFSET = 30;

// INTERAL HEADER FILES

#include <contest2/boxes.h>
#include <contest2/imagePipeline.h>
#include <contest2/navigation.h>
#include <contest2/robot_pose.h>
#include <contest2/state.h>

// MACROS

#define RAD2DEG(rad) ((rad) * 180. / M_PI) // Convert from radians to degrees
#define DEG2RAD(deg) ((deg) * M_PI / 180.) // Convert from degrees to radians

#endif // CONTEST2_H

#ifndef PATHFIND_H
#define PATHFIND_H

#include "contest1.h"
#include <limits>
#include <queue>

// Costmap node
struct Node {
  int idx;
  int cost;

  bool operator>(const Node &other) const { return cost > other.cost; }
};

std::vector<std::tuple<float, float>>
findPath(const nav_msgs::OccupancyGrid &grid, int startIdx, int goalIdx);

std::tuple<float, float> mapIdxToPos(std::tuple<int, int> gridIdx,
                                     float resolution, float originX,
                                     float originY);
int idxToRowMajor(std::tuple<int, int> gridIdx, int gridWidth);
std::tuple<int, int> rowMajorToIdx(int rowMajorIdx, int gridWidth);
#endif // PATHFIND_H

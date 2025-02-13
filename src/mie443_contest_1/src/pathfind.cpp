#include "pathfind.h"
#include "contest1.h"
#include "ros/console.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <tuple>

std::tuple<float, float> mapIdxToPos(std::tuple<int, int> gridIdx,
                                     float resolution, float originX,
                                     float originY) {
  float posX = (std::get<0>(gridIdx) * resolution) + originX;
  float posY = (std::get<1>(gridIdx) * resolution) + originY;

  return std::make_tuple(posX, posY);
}
int idxToRowMajor(std::tuple<int, int> gridIdx, int gridWidth) {
  return std::get<1>(gridIdx) * gridWidth + std::get<0>(gridIdx);
}

std::tuple<int, int> rowMajorToIdx(int rowMajorIdx, int gridWidth) {
  int x = rowMajorIdx % gridWidth;
  int y = rowMajorIdx / gridWidth;

  return std::make_tuple(x, y);
}

std::vector<int> calcObstDistCost(const nav_msgs::OccupancyGrid &grid) {
  ROS_INFO("Calculating obstacle distance costs");
  const int dx[4] = {
      0, 0,
      -1, // Left
      1   // Right
  };
  const int dy[4] = {-1, // Up
                     1,  // Down
                     0, 0};

  int width = grid.info.width;
  int height = grid.info.height;
  int totalCells = width * height;

  // Distance to obstacles
  std::vector<int> dist(totalCells, std::numeric_limits<int>::max());

  // Queue for indices of all detected obstacles
  std::queue<int> obst;

  // For all obstacles, set the distance to 0 and push to queue
  for (int idx = 0; idx < totalCells; idx++) {
    if (grid.data[idx] != 0) {
      dist[idx] = 0;
      obst.push(idx);
    }
  }

  while (!obst.empty()) {
    int cur = obst.front();
    obst.pop();

    std::tuple<int, int> xyIdx = rowMajorToIdx(cur, width);
    for (int i = 0; i < 4; i++) {
      int nx = std::get<0>(xyIdx) + dx[i];
      int ny = std::get<1>(xyIdx) + dy[i];

      // If out of bounds, skip
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      int nIdx = idxToRowMajor(std::make_tuple(nx, ny), width);
      if (dist[nIdx] > dist[cur] + 1) {
        dist[nIdx] = dist[cur] + 1;
        obst.push(nIdx);
      }
    }
  }

  return dist;
}

nav_msgs::OccupancyGrid inflateObstacles(const nav_msgs::OccupancyGrid &grid,
                                         int padding) {
  ROS_INFO("Adding padding to obstacles on map");
  nav_msgs::OccupancyGrid inflated = grid;
  int width = grid.info.width;
  int height = grid.info.height;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int idx = idxToRowMajor(std::make_tuple(x, y), width);

      for (int dy = -padding; dy <= padding; dy++) {
        for (int dx = -padding; dx <= padding; dx++) {
          int nx = x + dx;
          int ny = y + dy;

          if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            int nIdx = idxToRowMajor(std::make_tuple(nx, ny), width);
            inflated.data[nIdx] = 100;
          }
        }
      }
    }
  }

  return inflated;
}

std::vector<std::tuple<float, float>>
findPath(const nav_msgs::OccupancyGrid &grid, int startIdx, int goalIdx) {

  std::vector<std::tuple<float, float>> path;
  const int dx[4] = {
      0, 0,
      -1, // Left
      1   // Right
  };
  const int dy[4] = {-1, // Up
                     1,  // Down
                     0, 0};

  int width = grid.info.width;
  int height = grid.info.height;
  int totalCells = width * height;

  if (startIdx < 0 || startIdx >= totalCells || goalIdx < 0 ||
      goalIdx >= totalCells) {
    ROS_ERROR("The goal or start index is not valid: startIdx: %i, goalIdx: "
              "%i, totalCells: %i",
              startIdx, goalIdx, totalCells);
    return path;
  }

  // Vector of all the cells. Setting the distances to inf until calculated.
  std::vector<int> cellDist(totalCells, std::numeric_limits<int>::max());
  cellDist[startIdx] = 0;

  // Vector of obstacle distance costs
  std::vector<int> obstDist = calcObstDistCost(grid);

  // Vector of the parent cell indices. Setting to -1 until calculated.
  std::vector<int> prevCellIdx(totalCells, -1);

  // Always keep the nodes sorted by lowest cost.
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> nodeQueue;
  nodeQueue.push(
      Node{startIdx, 0}); // Init the node queue with the starting point

  while (!nodeQueue.empty()) {
    Node currentNode = nodeQueue.top();
    nodeQueue.pop(); // Remove so it's not checked again

    // We have reached the goal
    if (currentNode.idx == goalIdx) {
      break;
    }

    // There is already a shorter path, skip this cell
    if (currentNode.cost > cellDist[currentNode.idx]) {
      continue;
    }

    // Convert the row-major idx to x,y
    std::tuple<int, int> xyIdx = rowMajorToIdx(currentNode.idx, width);

    // Check around the current node for a valid neighbor
    for (int i = 0; i < 4; i++) {
      int nx = std::get<0>(xyIdx) + dx[i];
      int ny = std::get<1>(xyIdx) + dy[i];

      // Check if the test node is in bounds
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      int nIdx = idxToRowMajor(std::make_tuple(nx, ny), width);

      // Check if the cell is valid
      if (grid.data[nIdx] != 0) {
        continue;
      }

      // Skip if cell is too close to obstacle
      if (obstDist[nIdx] < OBSTACLE_PADDING) {
        continue;
      }

      // Update the cost of the cell if the cost of moving to the neighbor is
      // less than the distance. A shorter path has been found.
      int newCost = currentNode.cost + NEIGHBOR_COST;
      if (newCost < cellDist[nIdx]) {
        cellDist[nIdx] = newCost;
        prevCellIdx[nIdx] = currentNode.idx;
        nodeQueue.push(Node{nIdx, newCost}); // Add to node list to check later
      }
    }
  }

  // If the distance to the goal is still inf, a valid path wasn't found
  if (cellDist[goalIdx] == std::numeric_limits<int>::max()) {
    ROS_ERROR("No path was found from start to goal");
    return path;
  }

  // Reconstruct the shortest path
  for (int cellIdx = goalIdx; cellIdx != -1; cellIdx = prevCellIdx[cellIdx]) {
    std::tuple<int, int> waypointIdx = rowMajorToIdx(cellIdx, width);
    std::tuple<float, float> waypointPos =
        mapIdxToPos(waypointIdx, grid.info.resolution,
                    grid.info.origin.position.x, grid.info.origin.position.y);

    ROS_INFO("Added waypoint: (%f, %f)", std::get<0>(waypointPos),
             std::get<1>(waypointPos));

    path.push_back(waypointPos);
  }

  std::reverse(path.begin(),
               path.end()); // Flip the order so it starts at the start point

  ROS_INFO("Found a valid path to the goal");
  return path;
}

#include <cmath>
#include <contest2/contest2.h>

bool cmdOptionExists(char **begin, char **end, const std::string &option) {
  return std::find(begin, end, option) != end;
}

int main(int argc, char **argv) {
  // Parse args
  bool showView = true;
  if (cmdOptionExists(argv, argv + argc, "-hideView")) {
    showView = false;
  }

  // Setup ROS.
  ros::init(argc, argv, "contest2");
  ros::NodeHandle n;

  // Robot state object + subscriber.
  RobotState robotState(n);
  ros::Subscriber amclSub = n.subscribe(
      "/amcl_pose", 1, &RobotPose::poseCallback, &robotState.currPose);

  // Initialize box coordinates and templates
  Boxes boxes;
  if (!boxes.load_coords() || !boxes.load_templates()) {
    std::cout << "ERROR: could not load coords or templates" << std::endl;
    return -1;
  }
  for (int i = 0; i < boxes.coords.size(); ++i) {
    std::cout << "Box coordinates: " << std::endl;
    std::cout << i << " x: " << boxes.coords[i][0]
              << " y: " << boxes.coords[i][1] << " z: " << boxes.coords[i][2]
              << std::endl;
  }
  robotState.boxes = boxes;

  // contest count down timer
  std::chrono::time_point<std::chrono::system_clock> start;
  start = std::chrono::system_clock::now();
  uint64_t secondsElapsed = 0;

  // Execute strategy.
  while (ros::ok() && secondsElapsed <= 300) {
    ros::spinOnce();

    robotState.updateState(showView);

    ros::Duration(0.01).sleep();
  }
  return 0;
}

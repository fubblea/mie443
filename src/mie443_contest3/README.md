# Contest 3

## First-time setup

1. `sudo chmod +x ./src/turtlebot_follower/cfg/Follower.cfg`
2. `sudo chmod +x ./src/turtlebot_follower/scripts/switch.py`

## Running in Gazebo

1. `roslaunch turtlebot_gazebo turtlebot_world.launch`
2. `rosrun sound_play soundplay_node.py`
3. `rosrun mie443_contest3 contest3`

## Running in Real Life

1. `roslaunch turtlebot_bringup minimal.launch`
2. `rosrun sound_play soundplay_node.py`
3. `roslaunch turtlebot_follower follower.launch`
  - Make sure the person who you want to follow is in front of the robot at the required following distance before running this command.
4. `rosrun mie443_contest3 contest3`

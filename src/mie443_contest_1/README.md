# Contest 1

## Usage

Always `source devel/setup.bash` a new terminal. If the `devel` folder isn't there, you probably need to run `catkin_make`.

Always run `catkin_make` after any code changes.

### Gazebo

1. Start Gazebo: `roslaunch mie443_contest1 turtlebot_world.launch world:=1`
2. Start mapping: `roslaunch turtlebot_gazebo gmapping_demo.launch`
3. Start RViz: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
4. Start custom node: `rosrun mie443_contest1 contest1`

### Physical

1. Start Kobuki: `roslaunch turtlebot_bringup minimal.launch`
2. Start mapping: `roslaunch turtlebot_navigation gmapping_demo.launch`
3. Start RViz: `roslaunch turtlebot_rviz_launchers view_navigation.launch`
4. Start custom node: `rosrun mie443_contest1 contest1`
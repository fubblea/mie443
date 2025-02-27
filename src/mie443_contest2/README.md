# Contest 2

## Running in Sim

1. `roslaunch mie443_contest2 turtlebot_world.launch world:=1`
2. `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/thursday2023/mie443/src/mie443_contest2/maps/map_1.yaml`
  - If this is not running on the turtlebot laptop, then change the home path to the correct user. For example `/home/ajay/...`
3. `roslaunch turtlebot_rviz_launchers view_navigation.launch`
  - Set the original pose estimate (has to be somewhat accurate)
4. `rosrun mie443_contest2 contest2`
  - If the view window is ruining the fps, run: `rosrun mie443_contest2 contest2 -hideView`


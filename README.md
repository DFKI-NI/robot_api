# Robot API

A pre- and concise Python API to control robots with simple commands.

## Demo

Assuming you have the [mobipick](https://git.ni.dfki.de/mobipick/mobipick) repository installed and compiled, first start up a ROS environment, e.g.:
```
roslaunch mobipick_gazebo mobipick_moelk.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/moelk/pbr_robot_lab.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
roslaunch mobipick_pick_n_place moveit_macros.launch
```

To use this `robot_api`, just clone it somewhere, go into its `robot_api` folder and use a `python` console:
```
import robot_api
# Get a Robot object using the robot's namespace.
mobipick = robot_api.Robot("mobipick")
# Get the robot's 2D pose using localization.
mobipick.base.get_2d_pose()
# Move the robot's arm using MoveIt.
mobipick.arm.move("transport")
# Move the robot's base using move_base.
mobipick.base.move(21.0, 7.0, 3.141592)
```

## Discussion

The idea is not new at all. See for example Facebook's [PyRobot](https://pyrobot.org/) or our repository of [high_level_robot_api](https://git.ni.dfki.de/acting/high_level_robot_api/-/tree/noetic/src/high_level_robot_api). A few design decisions merit an own software package:

### Typed interfaces

When defining an interface, you might as well make it elaborate, even when it's Python. You will notice the difference when using an IDE with code completion or static type checking. `*args` and `**kwargs` are universal parameters but might require more of trail and error from the user than necessary.

### ROS independence

At least on user side, you don't need to worry about importing `rospy` or initializing a ROS node to use the `robot_api`. Beneath it, ROS is used indeed since a running ROS environment is a prerequisite in the first place. In the future, ROS 2 shall be supported as well. Under these circumstances, it makes sense for `robot_api` to accept ROS message types as parameters anyway. A detailed example is `Base.move()`, which accepts several variations of ROS and non-ROS parameters.

### Class structure

Some considerations have been made about whether you might prefer to type `robot.move_base()` over `robot.base.move()`. In the end, the syntax is chosen to adhere to a meaningful component model. A base with localization and navigation capabilities is assumed to exist for any robot, and its functionality is encapsulated in a `Robot()`'s `base` variable. Additional components are available as separate field variables of `Robot()`. The interface definition at this point is still work in progress.

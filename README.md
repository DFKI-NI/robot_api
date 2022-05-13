# Robot API

A pre- and concise Python API to control robots with simple commands.

## Overview

The Robot API is an interface layer between an executor and ROS robot control. A user shall have a simple and well defined (typed) interface to call to for the execution of robot actions. The goal is to enable control of our robots independently from their types and without knowledge of their operation systems.

On developer side, care is to be taken of the genericity of these actions by providing robot specific implementations towards this API. Special attention is necessary for capabilities which are not available on all robots. In these cases, the execution shall fail gracefully and not crash the system.

## Installation

Clone this repository into your catkin workspace and `catkin build` as usual. Source the resulting `setup.bash` file of your built catkin workspace afterwards.

## Demo

Assuming you have the [mobipick](https://github.com/DFKI-NI/mobipick) repository installed and compiled, first start up a ROS environment, e.g.:
```
roslaunch mobipick_gazebo mobipick_moelk.launch
rosservice call /gazebo/unpause_physics   # or click the "start" button in the Gazebo GUI
roslaunch mir_gazebo fake_localization.launch __ns:="mobipick" odom_frame_id:="mobipick/odom" base_frame_id:="mobipick/base_footprint"
roslaunch mir_navigation start_planner.launch map_file:=$(rospack find pbr_maps)/maps/moelk/pbr_robot_lab.yaml prefix:="mobipick/"
roslaunch mobipick_moveit_config moveit_planning_execution.launch use_pointcloud:=true simulation:=true
```

To use this `robot_api`, just install it as described above and use a `python` console anywhere:
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

### ROS wrapper

At least on user side, you don't need to worry about importing `rospy` or initializing a ROS node to use the `robot_api`. Beneath it, ROS is used indeed since a running ROS environment is a prerequisite in the first place. In the future, ROS 2 shall be supported as well. Under these circumstances, it makes sense for `robot_api` to accept ROS message types as parameters anyway. A detailed example is `Base.move()`, which accepts several variations of ROS and non-ROS parameters.

### Class structure

Some considerations have been made about whether you might prefer to type `robot.move_base()` over `robot.base.move()`. In the end, the syntax is chosen to adhere to a meaningful component model. A base with localization and navigation capabilities is assumed to exist for any robot, and its functionality is encapsulated in a `Robot()`'s `base` variable. Additional components are available as separate field variables of the `Robot` class. The interface definition at this point is still work in progress.

## Contribution

Your contribution is very welcome. Much robot specific functionality is still missing for many of our robots. Please develop on the `develop` branch or a sub-branch of it, and use merge requests to merge stable implementation back.

### Actions

All actions of all robots shall inherit from the Action class and implement an `execute()` static method. Preferably, each action has only one specific purpose. Convenience methods are available as class methods of `Base` or extensions, and they delegate action execution to specific Action implementations based on the parameter values. Again, `Base.move()` demonstrates a sample implementation of this concept.

### Extensions

Feel free to add further extensions as fields to the `Robot` class but keep in mind that these will exist then for all robots, even if they might not have the respective components. For example, `Base.arm` is such an extension. A gripper should not be an extension of `Base` unless we one day actually have a gripper on a robot's base without an arm. It could be an extension of `Base.arm` if you see benefits in encapsulating it further.

### Connections

By default, `robot_api` connects components lazily on demand. This means, for example, `actionlib` clients connect to their server when they execute something for the first time. Such mechnamisms are integral parts of the Robot API in order to offer convenience to the user. The user does not need to know about `actionlib` at all.

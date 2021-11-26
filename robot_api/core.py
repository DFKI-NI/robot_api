#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Callable, Optional, Sequence, Tuple
import time
import rospy
import tf
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from robot_api.extensions import Arm, MoveItMacrosArm
from robot_api.excepthook import Excepthook
from robot_api.lib import _init_node, Action, ActionlibComponent, Storage


class BaseMoveToGoalAction(Action):
    @staticmethod
    def execute(base: Base, goal: MoveBaseGoal, timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        if not base.connect_once("move_base"):
            rospy.logerr(f"Cannot move base to goal.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return

        p, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
        position = [p.x, p.y, p.z]
        orientation = [q.x, q.y, q.z, q.w]

        # Add waypoint if new, and move to goal.
        is_new_goal = (position, orientation) not in Storage.waypoints.values()
        custom_goal_name = Storage._get_custom_waypoint_name(position, orientation)
        rospy.logdebug(f"Sending {'new ' if is_new_goal else ''}navigation goal " \
            + (f"'{custom_goal_name}' " if custom_goal_name else "")
            + f"{(position, orientation)} ...")
        if is_new_goal:
            Storage._add_generic_waypoint(position, orientation)
        if done_cb is None:
            rospy.logdebug(f"Waiting for navigation result with timeout of {timeout} s ...")
            return base._action_clients["move_base"].send_goal_and_wait(goal, rospy.Duration(timeout))
        else:
            return base._action_clients["move_base"].send_goal(goal, done_cb)


class BaseMoveToPoseAction(Action):
    @staticmethod
    def execute(base: Base, pose: Pose, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return BaseMoveToGoalAction.execute(base, goal, timeout, done_cb)


class BaseMoveToPositionOrientationAction(Action):
    @staticmethod
    def execute(base: Base, position: Sequence[float], orientation: Sequence[float], frame_id: str="map",
            timeout: float=60.0, done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        assert len(position) == 3, "Parameter 'position' must have len 3."
        assert len(orientation) == 4, "Parameter 'orientation' must have len 4."
        pose = Pose()
        p, q = pose.position, pose.orientation
        p.x, p.y, p.z = position
        q.x, q.y, q.z, q.w = orientation
        return BaseMoveToPoseAction.execute(base, pose, frame_id, timeout, done_cb)


class BaseMoveToCoordinatesAction(Action):
    @staticmethod
    def execute(base: Base, x: float, y: float, yaw: float, pitch: float=0.0, roll: float=0.0, z: float=0.0,
            frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        position = (x, y, z)
        orientation = list(tf.transformations.quaternion_from_euler(roll, pitch, yaw))
        return BaseMoveToPositionOrientationAction.execute(base, position, orientation, frame_id, timeout, done_cb)


class Base(ActionlibComponent):
    def __init__(self, robot: Robot, connect_navigation_on_init: bool) -> None:
        super().__init__(robot.namespace, {"move_base": MoveBaseAction}, connect_navigation_on_init)
        self.robot = robot
        self._tf_listener = tf.TransformListener()

    def get_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[Sequence[float], Sequence[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            position, orientation = self._tf_listener.lookupTransform(reference_frame,
                self.robot.namespace + robot_frame, rospy.Time(0))
        except (tf.LookupException, tf.ExtrapolationException) as e:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        position, orientation = self._tf_listener.lookupTransform(reference_frame,
                            self.robot.namespace + robot_frame, rospy.Time(0))
                        return position, orientation
                    except tf.LookupException:
                        pass
            raise Excepthook.expect(e)

        return position, orientation

    def get_2d_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_pose(reference_frame, robot_frame, timeout)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

    def move_to_waypoint(self, name: str, frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to waypoint by name in frame_id's map with timeout.
        Optionally, call done_cb() afterwards if given.
        """
        if name not in Storage.waypoints.keys():
            if Storage.waypoints:
                rospy.logerr(f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + Storage._waypoints_to_str())
            else:
                rospy.logerr(f"No waypoints defined yet, so cannot use waypoint '{name}'.")
            return

        position, orientation = Storage.waypoints[name]
        return BaseMoveToPositionOrientationAction.execute(self, position=position, orientation=orientation,
            frame_id=frame_id, timeout=timeout, done_cb=done_cb)

    def move(self, x: Optional[float]=None, y: Optional[float]=None, yaw: Optional[float]=None,
            pitch: float=0.0, roll: float=0.0, z: float=0.0, position: Sequence[float]=[],
            orientation: Sequence[float]=[], pose: Optional[Pose]=None, goal: Optional[MoveBaseGoal]=None,
            frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to goal pose using the following parameter options in descending priority:
        move(goal: MoveBaseGoal)
        move(pose: Pose)
        move(position: Sequence[float], orientation: Sequence[float])
        move(x: float, y: float: yaw: float, pitch: float=0.0, roll: float=0.0, z: float=0.0)

        Optionally, call done_cb() afterwards if given.
        """
        if goal is None:
            if pose is None:
                if not position or not orientation:
                    if not position and not orientation:
                        if x is None or y is None or yaw is None:
                            raise Excepthook.expect(ValueError("1. Goal, 2. pose, 3. position and orientation,"
                                " or 4. x, y, and yaw must be specified."))
                        return BaseMoveToCoordinatesAction.execute(self, x, y, yaw, pitch, roll, z, frame_id, timeout,
                            done_cb)
                    else:
                        # Raise error if only one of position and orientation is specified.
                        raise Excepthook.expect(ValueError("Both 'position' and 'orientation' parameters"
                            " must be specified."))
                return BaseMoveToPositionOrientationAction.execute(self, position, orientation, frame_id, timeout,
                    done_cb)
            return BaseMoveToPoseAction.execute(self, pose, frame_id, timeout, done_cb)
        return BaseMoveToGoalAction.execute(self, goal, timeout, done_cb)


class Robot:
    def __init__(self, namespace: str=rospy.get_namespace(), connect_navigation_on_init: bool=False,
            connect_manipulation_on_init: bool=False) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace
        self.base = Base(self, connect_navigation_on_init)
        self.arm = MoveItMacrosArm(self.namespace, connect_manipulation_on_init) \
            if ActionlibComponent._is_topic_of_type(self.namespace, "moveit_macros/result", "MoveItMacroActionResult") else Arm(self.namespace, connect_manipulation_on_init)

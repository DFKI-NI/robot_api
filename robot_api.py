#!/usr/bin/env python3
from __future__ import annotations
from typing import Any, Callable, Dict, List, Optional, Tuple
import os
import time
import rospy
import rosnode
import rosgraph
import tf
import actionlib
import re
import yaml
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult
from geometry_msgs.msg import Pose
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult
from excepthook import expect_exception


def _init_node() -> None:
    """Initialize ROS node for the current process if not already done."""
    if not rosgraph.is_master_online():
        rospy.logdebug("Waiting for ROS master node to go online ...")
        while not rosgraph.is_master_online():
            time.sleep(1.0)
    if rospy.is_shutdown():
        rospy.logerr("ROS is shutting down.")
        return

    name = f"robot_api_{os.getpid()}"
    # Note: ROS cannot check if a node is initialized, so we have to try.
    #   At least check if we initialized it ourselves before.
    if ('/' + name) not in rosnode.get_node_names():
        try:
            rospy.init_node(name)
            rospy.logdebug(f"ROS node '{name}' initialized.")
        except rospy.ROSException:
            pass


def _s(count: int, name: str, plural: str='s') -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"


def find_robot_namespaces() -> List[str]:
    """Return list of robot namespaces by searching published topics for move_base/goal."""
    topics = rospy.get_published_topics()
    robot_namespaces = []  # type: List[str]
    for topic, message_type in topics:
        if message_type == "move_base_msgs/MoveBaseActionGoal":
            match_result = re.match(r'\/(\w+)\/move_base\/goal', topic)
            if match_result:
                robot_namespaces.append(match_result.group(1))
    return robot_namespaces


class Robot:
    def __init__(self, namespace: str=rospy.get_namespace(),
            connect_navigation_on_init: bool=False) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace
        self._tf_listener = tf.TransformListener()
        self._move_base_action_client = actionlib.SimpleActionClient(self.namespace + "move_base", MoveBaseAction)
        self._moveit_macros_action_client = actionlib.SimpleActionClient(self.namespace + "moveit_macros",
            MoveItMacroAction)
        self.waypoints = OrderedDict()  # type: Dict[str, Tuple[List[float], List[float]]]
        self._next_waypoint = 1
        self._executed_methods = {}  # type: Dict[Callable[[], Any], Any]
        if connect_navigation_on_init:
            self._execute_once(self._connect_move_base)

    def _execute_once(self, method: Callable[[], Any]) -> Any:
        if method not in self._executed_methods.keys():
            self._executed_methods[method] = method()
        return self._executed_methods[method]

    def _connect_move_base(self) -> Any:
        rospy.logdebug("Waiting for move_base action server ...")
        return self._move_base_action_client.wait_for_server()

    def _connect_moveit_macros(self) -> Any:
        rospy.logdebug("Waiting for moveit_macros action server ...")
        return self._moveit_macros_action_client.wait_for_server()

    def _add_generic_waypoint(self, position: List[float], orientation: List[float]) -> None:
        """Add (position, orientation) with generic name to list of stored waypoints."""
        while "waypoint" + str(self._next_waypoint) in self.waypoints.keys():
            self._next_waypoint += 1
        self.waypoints["waypoint" + str(self._next_waypoint)] = (position, orientation)

    def _waypoints_to_str(self) -> str:
        """Convert OrderedDict representation of waypoints to str."""
        return '\n'.join(f"'{waypoint_name}': {waypoint}" for waypoint_name, waypoint in self.waypoints.items())

    def _get_custom_waypoint_name(self, position: List[float], orientation: List[float]) -> str:
        """Return custom name of waypoint (position, orientation) if it exists."""
        for name, waypoint in self.waypoints.items():
            if not name.startswith("waypoint") and waypoint == (position, orientation):
                return name
        return ""

    def get_base_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[List[float], List[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            position, orientation = self._tf_listener.lookupTransform(reference_frame,
                self.namespace + robot_frame, rospy.Time(0))
        except tf.LookupException as e:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        position, orientation = self._tf_listener.lookupTransform(reference_frame,
                            self.namespace + robot_frame, rospy.Time(0))
                        return position, orientation
                    except tf.LookupException:
                        pass
            expect_exception(e)
            raise
        return position, orientation

    def get_base_2d_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_base_pose(reference_frame, robot_frame, timeout)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

    def move_base(self, x: Optional[float]=None, y: Optional[float]=None, yaw: Optional[float]=None,
            pitch: float=0.0, roll: float=0.0, z: float=0.0, position: List[float]=[],
            orientation: List[float]=[], pose: Optional[Pose]=None, goal: Optional[MoveBaseGoal]=None,
            frame_id: str="map", timeout: float=60.0,
            done_cb: Optional[Callable[[int, MoveBaseResult], Any]]=None) -> Any:
        """Move robot to goal pose using the following parameter options in descending priority:
        move_base(goal: MoveBaseGoal)
        move_base(pose: Pose)
        move_base(position: List[float], orientation: List[float])
        move_base(x: float, y: float: yaw: float, pitch: float=0.0, roll: float=0.0, z: float=0.0)
        """
        if not self._execute_once(self._connect_move_base):
            rospy.logerr(f"Cannot move base to goal.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return

        if goal is None:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = frame_id
            goal.target_pose.header.stamp = rospy.Time.now()
            if pose is None:
                if not position:
                    if x is None or y is None:
                        raise expect_exception(ValueError("Goal, pose, position, or both x and y must be specified."))
                    position = [x, y, z]
                elif len(position) != 3:
                    raise expect_exception(ValueError("Parameter position must have len 3."))
                if not orientation:
                    if yaw is None:
                        raise expect_exception(ValueError("Goal, pose, orientation, or yaw angle must be specified."))
                    orientation = list(tf.transformations.quaternion_from_euler(roll, pitch, yaw))
                elif len(orientation) != 4:
                    raise expect_exception(ValueError("Parameter orientation must have len 4."))
                p, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
                p.x, p.y, p.z = position
                q.x, q.y, q.z, q.w = orientation
            else:
                goal.target_pose.pose = pose
                p, q = pose.position, pose.orientation
                position = [p.x, p.y, p.z]
                orientation = [q.x, q.y, q.z, q.w]
        else:
            p, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
            position = [p.x, p.y, p.z]
            orientation = [q.x, q.y, q.z, q.w]

        is_new_goal = (position, orientation) not in self.waypoints.values()
        custom_goal_name = self._get_custom_waypoint_name(position, orientation)
        rospy.logdebug(f"Sending {'new ' if is_new_goal else ''}navigation goal " \
            + (f"'{custom_goal_name}' " if custom_goal_name else "")
            + f"{(position, orientation)} ...")
        if is_new_goal:
            self._add_generic_waypoint(position, orientation)
        if done_cb is None:
            rospy.logdebug(f"Waiting for navigation result with timeout of {timeout} s ...")
            return self._move_base_action_client.send_goal_and_wait(goal, rospy.Duration(timeout))
        else:
            return self._move_base_action_client.send_goal(goal, done_cb)

    def move_to_waypoint(self, name: str, frame_id: str="map", timeout: float=60.0) -> Any:
        """Move robot to waypoint by name in frame_id's map with timeout."""
        if name not in self.waypoints.keys():
            if self.waypoints:
                rospy.logerr(f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + self._waypoints_to_str())
            else:
                rospy.logerr(f"No waypoints defined yet, so cannot use waypoint '{name}'.")
            return

        position, orientation = self.waypoints[name]
        return self.move_base(position=position, orientation=orientation,
            frame_id=frame_id, timeout=timeout)

    def add_waypoint(self, name: str, position: List[float], orientation: List[float]) -> None:
        """Add waypoint with name, position [x, y, z] and orientation [x, y, z, w]."""
        if name in self.waypoints.keys():
            rospy.logwarn(f"Overwriting waypoint: {self.waypoints[name]}")
        self.waypoints[name] = (position, orientation)

    def save_waypoints(self, filepath: str="~/.ros/robot_api_waypoints.txt") -> None:
        """Save waypoints to file, default: '~/.ros/robot_api_waypoints.txt'."""
        if not self.waypoints:
            rospy.logwarn("No waypoints to save.")
            return

        filepath = os.path.expanduser(filepath)
        try:
            with open(filepath, 'w') as text_file:
                for waypoint_name, waypoint in self.waypoints.items():
                    # Note: Write tuple as list for nicer format.
                    text_file.write(f"'{waypoint_name}': {list(waypoint)}\n")
        except Exception:
            rospy.logerr(f"Cannot write to file '{filepath}'!")

    def load_waypoints(self, filepath: str="~/.ros/robot_api_waypoints.txt") -> None:
        """Load waypoints from file, default: '~/.ros/robot_api_waypoints.txt'."""
        filepath = os.path.expanduser(filepath)
        try:
            with open(filepath, 'r') as text_file:
                lines = [line.strip() for line in text_file.readlines()]
            for line in lines:
                elements = yaml.safe_load(line)
                assert isinstance(elements, dict), f"Invalid format in line: {line}"
                for name, (position, orientation) in elements.items():
                    self.add_waypoint(name, position, orientation)
            rospy.loginfo(f"{_s(len(lines), 'waypoint')} loaded, now {len(self.waypoints)} in total.")
        except Exception:
            rospy.logerr(f"Cannot read from file '{filepath}'!")

    def print_waypoints(self) -> None:
        """Output currently stored waypoints with rospy.loginfo()."""
        rospy.loginfo(f"Available waypoints:\n" + self._waypoints_to_str())

    def move_arm(self, goal_name: str, goal_type: str="target",
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to goal_name if goal_type is 'target', or call goal_name if goal_type is 'function'.
        Optionally, call done_cb() afterwards if given.
        """
        goal = MoveItMacroGoal()
        goal.type = goal_type
        goal.name = goal_name
        if not self._execute_once(self._connect_moveit_macros):
            rospy.logerr(f"Cannot move arm to goal.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return

        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        if done_cb is None:
            return self._moveit_macros_action_client.send_goal_and_wait(goal)
        else:
            return self._moveit_macros_action_client.send_goal(goal, done_cb)

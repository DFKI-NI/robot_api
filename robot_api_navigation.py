#!/usr/bin/env python3
from __future__ import annotations
from typing import TYPE_CHECKING, Any, Dict, List, Tuple
import os
import rospy
import tf
import actionlib
import yaml
from collections import OrderedDict
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
if TYPE_CHECKING:  # Resolve circular dependency while type checking.
    from robot_api import Robot


def _s(count: int, name: str, plural: str='s') -> str:
    """Return name with or without plural ending depending on count."""
    return f"{count} {name}{plural if count != 1 else ''}"


class Navigation:
    def __init__(self, robot: Robot, connect: bool) -> None:
        self._robot = robot
        self._action_client = actionlib.SimpleActionClient(robot.namespace + "move_base", MoveBaseAction)
        self._waypoints = OrderedDict()  # type: Dict[str, Tuple[List[float], List[float]]]
        self._next_waypoint = 1
        self._connected = False
        if connect:
            self._connect()

    def _connect(self) -> bool:
        """Connect to move_base action server if not already connected."""
        if not self._connected:
            rospy.loginfo("Waiting for move_base action server ...")
            if self._action_client.wait_for_server():
                self._connected = True
        return self._connected

    def _add_generic_waypoint(self, position: List[float], orientation: List[float]) -> None:
        """Add (position, orientation) with generic name to list of stored waypoints."""
        while "waypoint" + str(self._next_waypoint) in self._waypoints.keys():
            self._next_waypoint += 1
        self._waypoints["waypoint" + str(self._next_waypoint)] = (position, orientation)

    def _waypoints_to_str(self) -> str:
        """Convert OrderedDict representation of waypoints to str."""
        return '\n'.join(f"'{waypoint_name}': {waypoint}" for waypoint_name, waypoint in self._waypoints.items())

    def _get_custom_waypoint_name(self, position: List[float], orientation: List[float]) -> str:
        """Return custom name of waypoint (position, orientation) if it exists."""
        for name, waypoint in self._waypoints.items():
            if not name.startswith("waypoint") and waypoint == (position, orientation):
                return name

        return ""

    def move_to_goal(self, goal: MoveBaseGoal, timeout: float) -> Any:
        """Move robot to move_base_msgs/MoveBaseGoal with timeout."""
        if not self._connect():
            rospy.logerr("Cannot move to goal.")
            return

        p, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
        position = [p.x, p.y, p.z]
        orientation = [q.x, q.y, q.z, q.w]
        is_new_goal = (position, orientation) not in self._waypoints.values()
        custom_goal_name = self._get_custom_waypoint_name(position, orientation)
        rospy.loginfo(f"Sending {'new ' if is_new_goal else ''}navigation goal " \
            + (f"'{custom_goal_name}' " if custom_goal_name else "")
            + f"{(position, orientation)} ...")
        self._action_client.send_goal(goal)
        if is_new_goal:
            self._add_generic_waypoint(position, orientation)
        rospy.loginfo(f"Waiting for navigation result with timeout of {timeout} s ...")
        if not self._action_client.wait_for_result(rospy.Duration(timeout)):
            rospy.logwarn("Canceling goal ...")
            self._action_client.cancel_goal()
            rospy.logerr(f"An error occurred while waiting for navigation action server result.")
            return None

        return self._action_client.get_result()

    def move_to_pose(self, pose: Pose, frame_id: str="map", timeout: float=60.0) -> Any:
        """Move robot to geometry_msgs/Pose in frame_id's map with timeout."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose
        return self.move_to_goal(goal, timeout)

    def move_to_2d_pose(self, x: float, y: float, yaw: float=0.0, frame_id: str="map", timeout: float=60.0) -> Any:
        """Move robot to pose (x, y, yaw in radians) in frame_id's map with timeout."""
        pose = Pose()
        pose.position.x, pose.position.y = x, y
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w \
            = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
        return self.move_to_pose(pose, frame_id, timeout)

    def move_to_position_and_orientation(self, position: List[float], orientation: List[float],
            frame_id: str="map", timeout: float=60.0) -> Any:
        """Move robot to position [x, y, z] and orientation [x, y, z, w] in frame_id's map with timeout."""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation
        return self.move_to_pose(pose, frame_id, timeout)

    def move_to_waypoint(self, name: str, frame_id: str="map", timeout: float=60.0) -> Any:
        """Move robot to waypoint by name in frame_id's map with timeout."""
        if name not in self._waypoints.keys():
            if self._waypoints:
                rospy.logerr(f"Waypoint '{name}' does not exist. Available waypoints:\n"
                    + self._waypoints_to_str())
            else:
                rospy.logerr(f"No waypoints defined yet, so cannot use waypoint '{name}'.")
            return

        position, orientation = self._waypoints[name]
        return self.move_to_position_and_orientation(position, orientation, frame_id, timeout)

    def add_waypoint(self, name: str, position: List[float], orientation: List[float]) -> None:
        """Add waypoint with name, position [x, y, z] and orientation [x, y, z, w]."""
        if name in self._waypoints.keys():
            rospy.logwarn(f"Overwriting waypoint: {self._waypoints[name]}")
        self._waypoints[name] = (position, orientation)

    def save_waypoints(self, filepath: str="~/.ros/robot_api_waypoints.txt") -> None:
        """Save waypoints to file, default: '~/.ros/robot_api_waypoints.txt'."""
        if not self._waypoints:
            rospy.logwarn("No waypoints to save.")
            return

        filepath = os.path.expanduser(filepath)
        try:
            with open(filepath, 'w') as text_file:
                for waypoint_name, waypoint in self._waypoints.items():
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
            rospy.loginfo(f"{_s(len(lines), 'waypoint')} loaded, now {len(self._waypoints)} in total.")
        except Exception:
            rospy.logerr(f"Cannot read from file '{filepath}'!")

    def print_waypoints(self) -> None:
        """Output currently stored waypoints with rospy.loginfo()."""
        rospy.loginfo(f"Available waypoints:\n" + self._waypoints_to_str())

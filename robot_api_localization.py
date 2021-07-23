#!/usr/bin/env python3
from __future__ import annotations
from typing import TYPE_CHECKING, List, Tuple
import rospy
import tf
if TYPE_CHECKING:  # Resolve circular dependency while type checking.
    from robot_api import Robot


class Localization:
    def __init__(self, robot: Robot, connect: bool) -> None:
        self._robot = robot
        self._tf_listener = tf.TransformListener()

    def get_pose(self, reference_frame: str="map", robot_frame: str="base_footprint") \
            -> Tuple[List[float], List[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            position, orientation = self._tf_listener.lookupTransform(reference_frame,
                self._robot.namespace + robot_frame, rospy.Time(0))
        except tf.LookupException as exc:
            rospy.logerr(exc)
            raise
        return position, orientation

    def get_2d_pose(self, reference_frame: str="map", robot_frame: str="base_footprint") \
            -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_pose(reference_frame, robot_frame)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

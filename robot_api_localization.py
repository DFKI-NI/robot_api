#!/usr/bin/env python3
from __future__ import annotations
from typing import TYPE_CHECKING, List, Tuple, Type
from types import TracebackType
import sys
import time
import rospy
import tf
if TYPE_CHECKING:  # Resolve circular dependency while type checking.
    from robot_api import Robot


# Use this custom excepthook to replace default exception message with traceback by rospy.logerr() once.
_sys_excepthook = sys.excepthook
def _excepthook(exception_type: Type[BaseException], exception_value: BaseException,
        traceback: TracebackType) -> None:
    rospy.logerr(f"{exception_type.__module__}.{exception_type.__name__}: {exception_value}")
    sys.excepthook = _sys_excepthook


class Localization:
    def __init__(self, robot: Robot, connect: bool) -> None:
        self._robot = robot
        self._tf_listener = tf.TransformListener()

    def get_pose(self, reference_frame: str="map", robot_frame: str="base_footprint", timeout: float=1.0) \
            -> Tuple[List[float], List[float]]:
        """Return robot pose as tuple of position [x, y, z] and orientation [x, y, z, w]."""
        try:
            position, orientation = self._tf_listener.lookupTransform(reference_frame,
                self._robot.namespace + robot_frame, rospy.Time(0))
        except tf.LookupException:
            # If timeout is given, repeatedly try again.
            if timeout:
                time_start = time.time()
                while time.time() - time_start < timeout:
                    try:
                        time.sleep(1.0)
                        position, orientation = self._tf_listener.lookupTransform(reference_frame,
                            self._robot.namespace + robot_frame, rospy.Time(0))
                        return position, orientation
                    except tf.LookupException:
                        pass
            sys.excepthook = _excepthook
            raise
        return position, orientation

    def get_2d_pose(self, reference_frame: str="map", robot_frame: str="base_footprint",
            timeout: float=1.0) -> Tuple[float, float, float]:
        """Return robot pose as (x, y, yaw in radians)."""
        position, orientation = self.get_pose(reference_frame, robot_frame, timeout)
        _, _, yaw = tf.transformations.euler_from_quaternion(orientation)
        return position[0], position[1], yaw

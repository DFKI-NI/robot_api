#!/usr/bin/env python3
from typing import List
import os
import time
import rospy
import rosnode
import rosgraph
import tf
from robot_api_navigation import Navigation
from robot_api_localization import Localization


def _init_node() -> None:
    """Initialize ROS node for the current process if not already done."""
    if not rosgraph.is_master_online():
        rospy.logwarn("Waiting for ROS master node to go online ...")
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
            rospy.loginfo(f"ROS node '{name}' initialized.")
        except rospy.ROSException:
            pass


def angle_to_quaternion(yaw: float) -> List[float]:
    """Transform yaw angle in radians to quaternion [x, y, z, w]."""
    return list(tf.transformations.quaternion_from_euler(0.0, 0.0, yaw))


class Robot:
    def __init__(self, namespace: str=rospy.get_namespace(), connect_navigation_on_init: bool=False,
            connect_localization_on_init: bool=False) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace
        self.navigation = Navigation(self, connect_navigation_on_init)
        self.localization = Localization(self, connect_localization_on_init)

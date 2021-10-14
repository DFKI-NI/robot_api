#!/usr/bin/env python3
from typing import Any, Callable, Optional
import rospy
import actionlib
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult
from robot_api.lib import execute_once


class Arm:
    def __init__(self, namespace: str) -> None:
        self._moveit_macros_action_client = actionlib.SimpleActionClient(namespace + "moveit_macros",
            MoveItMacroAction)

    def _connect_moveit_macros(self) -> Any:
        rospy.logdebug("Waiting for moveit_macros action server ...")
        return self._moveit_macros_action_client.wait_for_server()

    def move(self, goal_name: str, goal_type: str="target",
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to goal_name if goal_type is 'target', or call goal_name if goal_type is 'function'.
        Optionally, call done_cb() afterwards if given.
        """
        goal = MoveItMacroGoal()
        goal.type = goal_type
        goal.name = goal_name
        if not execute_once(self._connect_moveit_macros):
            rospy.logerr(f"Cannot move arm to goal.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return None

        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        if done_cb is None:
            return self._moveit_macros_action_client.send_goal_and_wait(goal)
        else:
            return self._moveit_macros_action_client.send_goal(goal, done_cb)

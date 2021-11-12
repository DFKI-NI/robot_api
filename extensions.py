#!/usr/bin/env python3
from typing import Any, Callable, Optional
import rospy
import actionlib
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, \
    FtObserverAction, FtObserverGoal
from robot_api.lib import execute_once


class Arm:
    def __init__(self, namespace: str) -> None:
        self._moveit_macros_action_client = actionlib.SimpleActionClient(namespace + "moveit_macros",
            MoveItMacroAction)
        self._ft_observer_action_client = actionlib.SimpleActionClient(namespace + "ft_observer", FtObserverAction)

    def _connect_moveit_macros(self) -> Any:
        rospy.logdebug("Waiting for moveit_macros action server ...")
        return self._moveit_macros_action_client.wait_for_server()

    def _connect_ft_observer(self) -> Any:
        rospy.logdebug("Waiting for ft_observer action server ...")
        return self._ft_observer_action_client.wait_for_server()

    def execute(self, action_name: str, action_type: str="function",
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro of action_type with action_name. Optionally, call done_cb() afterwards if given."""
        if not execute_once(self._connect_moveit_macros):
            rospy.logerr(f"Cannot execute MoveIt action.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return None

        goal = MoveItMacroGoal()
        goal.type = action_type
        goal.name = action_name
        if done_cb is None:
            return self._moveit_macros_action_client.send_goal_and_wait(goal)
        else:
            return self._moveit_macros_action_client.send_goal(goal, done_cb)

    def move(self, goal_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose with goal_name. Optionally, call done_cb() afterwards if given."""
        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        return self.execute(goal_name, "target", done_cb)

    def get_result(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute action_name and return its result (not state). Optionally, call done_cb() afterwards if given."""
        self.execute(action_name, "function", done_cb)
        return self._moveit_macros_action_client.get_result()

    def observe_user_interaction(self) -> Any:
        """Observe and return state."""
        execute_once(self._connect_ft_observer)
        goal = FtObserverGoal()
        goal.threshold = 5.0
        goal.timeout = 30
        self._ft_observer_action_client.send_goal(goal)
        self._ft_observer_action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return self._ft_observer_action_client.get_state() == 3

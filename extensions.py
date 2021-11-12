#!/usr/bin/env python3
from typing import Any, Callable, Optional
import rospy
from mobipick_pick_n_place.msg import MoveItMacroAction, MoveItMacroGoal, MoveItMacroResult, \
    FtObserverAction, FtObserverGoal
from robot_api.lib import Component


class Arm(Component):
    def __init__(self, namespace: str, connect_manipulation_on_init: bool) -> None:
        super().__init__(namespace, {
            "moveit_macros": MoveItMacroAction,
            "ft_observer": FtObserverAction
        }, connect_manipulation_on_init)

    def execute(self, action_name: str, action_type: str="function",
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro of action_type with action_name. Optionally, call done_cb() afterwards if given."""
        if not self.connect_once("moveit_macros"):
            rospy.logerr(f"Cannot execute MoveIt action.{' ROS is shutting down.' if rospy.is_shutdown() else ''}")
            return None

        goal = MoveItMacroGoal()
        goal.type = action_type
        goal.name = action_name
        if done_cb is None:
            return self._action_clients["moveit_macros"].send_goal_and_wait(goal)
        else:
            return self._action_clients["moveit_macros"].send_goal(goal, done_cb)

    def move(self, goal_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose with goal_name. Optionally, call done_cb() afterwards if given."""
        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        return self.execute(goal_name, "target", done_cb)

    def get_result(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute action_name and return its result (not state). Optionally, call done_cb() afterwards if given."""
        self.execute(action_name, "function", done_cb)
        return self._action_clients["moveit_macros"].get_result()

    def observe_user_interaction(self) -> Any:
        """Observe and return state."""
        self.connect_once("ft_observer")
        goal = FtObserverGoal()
        goal.threshold = 5.0
        goal.timeout = 30
        action_client = self._action_clients["ft_observer"]
        action_client.send_goal(goal)
        action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return action_client.get_state() == 3

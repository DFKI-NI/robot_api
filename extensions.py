#!/usr/bin/env python3
from typing import Any, Callable, Optional
import genpy
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

    def _call_move_it(self, goal_type: str, goal_name: str,
            done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        if not self.connect_once("moveit_macros"):
            rospy.logerr("Cannot connect to moveit_macros.")
            return None

        goal = MoveItMacroGoal(type=goal_type, name=goal_name)
        if done_cb is None:
            return self._action_clients["moveit_macros"].send_goal_and_wait(goal)
        else:
            return self._action_clients["moveit_macros"].send_goal(goal, done_cb)

    def execute(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute moveit_macro named action_name. Optionally, call done_cb() afterwards if given."""
        return self._call_move_it("function", action_name, done_cb)

    def get_result(self, action_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Execute action_name and return its result (not state). Optionally, call done_cb() afterwards if given."""
        self._call_move_it("function", action_name, done_cb)
        return self._action_clients["moveit_macros"].get_result()

    def move(self, goal_name: str, done_cb: Optional[Callable[[int, MoveItMacroResult], Any]]=None) -> Any:
        """Move arm to pose named goal_name. Optionally, call done_cb() afterwards if given."""
        rospy.logdebug(f"Sending moveit_macro goal '{goal_name}' ...")
        return self._call_move_it("target", goal_name, done_cb)

    def observe_force_torque(self, threshold: float, timeout: float) -> bool:
        """Read from force torque sensor using threshold and timeout, then return whether it was successful."""
        if not self.connect_once("ft_observer"):
            rospy.logerr("Cannot connect to ft_observer.")
            return False

        goal = FtObserverGoal(threshold=threshold, timeout=timeout)
        action_client = self._action_clients["ft_observer"]
        action_client.send_goal(goal)
        action_client.wait_for_result()
        # Note: http://docs.ros.org/en/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
        return int(action_client.get_state()) == 3
